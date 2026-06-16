//! Parser for ROS2 `.msg` definitions.
//!
//! The `.msg` format is line-oriented, so the top-level driver
//! ([`parse_message`]) splits the input into logical lines (stripping
//! quote-aware `#` comments) and hands each non-blank line to a `nom` parser
//! that yields either a [`Constant`] or a [`Field`]. Doing the line splitting
//! outside `nom` keeps comment/whitespace handling simple and lets us attach
//! 1-based line numbers to every error.

use nom::{
    IResult, Parser,
    branch::alt,
    bytes::complete::{tag, take_while, take_while1},
    character::complete::{char, digit1, multispace0, one_of, space0, space1},
    combinator::{all_consuming, map, map_res, opt, recognize, value},
    multi::separated_list0,
    number::complete::double,
    sequence::{delimited, preceded},
};

use crate::ast::*;

/// An error parsing a `.msg` definition, with the offending 1-based line.
#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    pub line: usize,
    pub content: String,
    pub message: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "parse error on line {}: {} (in `{}`)",
            self.line, self.message, self.content
        )
    }
}

impl std::error::Error for ParseError {}

/// Parse a complete `.msg` definition.
pub fn parse_message(input: &str) -> Result<MessageSpec, ParseError> {
    parse_numbered_lines(input.lines().enumerate().map(|(i, l)| (i + 1, l)))
}

/// Parse a `.srv` definition (request `---` response).
pub fn parse_service(input: &str) -> Result<ServiceSpec, ParseError> {
    let sections = split_sections(input);
    if sections.len() != 2 {
        return Err(section_count_error("service", 2, sections.len()));
    }
    Ok(ServiceSpec {
        request: parse_numbered_lines(sections[0].iter().copied())?,
        response: parse_numbered_lines(sections[1].iter().copied())?,
    })
}

/// Parse a `.action` definition (goal `---` result `---` feedback).
pub fn parse_action(input: &str) -> Result<ActionSpec, ParseError> {
    let sections = split_sections(input);
    if sections.len() != 3 {
        return Err(section_count_error("action", 3, sections.len()));
    }
    Ok(ActionSpec {
        goal: parse_numbered_lines(sections[0].iter().copied())?,
        result: parse_numbered_lines(sections[1].iter().copied())?,
        feedback: parse_numbered_lines(sections[2].iter().copied())?,
    })
}

fn section_count_error(kind: &str, want: usize, got: usize) -> ParseError {
    ParseError {
        line: 0,
        content: String::new(),
        message: format!("{kind} must have {want} sections separated by `---`, found {got}"),
    }
}

/// Split input into `---`-delimited sections, preserving 1-based line numbers.
fn split_sections(input: &str) -> Vec<Vec<(usize, &str)>> {
    let mut sections = vec![Vec::new()];
    for (idx, line) in input.lines().enumerate() {
        if strip_comment(line).trim() == "---" {
            sections.push(Vec::new());
        } else {
            sections.last_mut().unwrap().push((idx + 1, line));
        }
    }
    sections
}

fn parse_numbered_lines<'a>(
    lines: impl Iterator<Item = (usize, &'a str)>,
) -> Result<MessageSpec, ParseError> {
    let mut spec = MessageSpec::default();

    for (line_no, raw_line) in lines {
        let stripped = strip_comment(raw_line);
        if stripped.trim().is_empty() {
            continue;
        }

        match all_consuming(line_item).parse(stripped) {
            Ok((_, item)) => match item {
                LineItem::Constant(c) => spec.constants.push(c),
                LineItem::Field(field) => spec.fields.push(field),
            },
            Err(e) => {
                return Err(ParseError {
                    line: line_no,
                    content: stripped.trim().to_string(),
                    message: describe_nom_error(&e),
                });
            }
        }
    }

    Ok(spec)
}

fn describe_nom_error(e: &nom::Err<nom::error::Error<&str>>) -> String {
    match e {
        nom::Err::Error(inner) | nom::Err::Failure(inner) => {
            if inner.input.trim().is_empty() {
                "unexpected end of line".to_string()
            } else {
                format!("unexpected `{}`", inner.input.trim())
            }
        }
        nom::Err::Incomplete(_) => "incomplete input".to_string(),
    }
}

/// Remove a `#` line comment, respecting single- and double-quoted strings so a
/// `#` inside a string literal (default value or string constant) is preserved.
fn strip_comment(line: &str) -> &str {
    let bytes = line.as_bytes();
    let mut quote: Option<u8> = None;
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i];
        match quote {
            Some(q) => {
                if c == b'\\' {
                    i += 1; // skip escaped char
                } else if c == q {
                    quote = None;
                }
            }
            None => {
                if c == b'"' || c == b'\'' {
                    quote = Some(c);
                } else if c == b'#' {
                    return &line[..i];
                }
            }
        }
        i += 1;
    }
    line
}

enum LineItem {
    Constant(Constant),
    Field(Field),
}

/// A single non-blank line: a constant if a top-level `=` follows the name,
/// otherwise a field (with optional default).
fn line_item(input: &str) -> IResult<&str, LineItem> {
    let (input, _) = space0(input)?;
    let (input, base) = base_type(input)?;
    let (input, array) = opt(array_suffix).parse(input)?;
    let (input, _) = space1(input)?;
    let (after_name, name) = identifier(input)?;

    // Constant iff `=` follows the name (after optional spaces). Only scalar
    // base types reach here meaningfully; resolution rejects array constants.
    let (after_eq_ws, _) = space0(after_name)?;
    if let Ok((input, _)) = char::<_, nom::error::Error<&str>>('=').parse(after_eq_ws) {
        let (input, _) = space0(input)?;
        let (input, val) = literal(&base, array.is_some())(input)?;
        let (input, _) = space0(input)?;
        return Ok((
            input,
            LineItem::Constant(Constant {
                ty: base,
                name,
                value: val,
            }),
        ));
    }

    let ty = FieldType { base, array };
    // Optional default value: anything non-blank following the name (separated
    // by at least one space).
    let (input, default) = if after_name.trim().is_empty() {
        (after_name, None)
    } else {
        let (input, _) = space1(after_name)?;
        let (input, val) = literal(&ty.base, ty.array.is_some())(input)?;
        (input, Some(val))
    };
    let (input, _) = space0(input)?;
    Ok((input, LineItem::Field(Field { ty, name, default })))
}

/// A base (element) type, before any array suffix.
fn base_type(input: &str) -> IResult<&str, BaseType> {
    alt((
        string_type,
        builtin_temporal_type,
        primitive_type,
        named_type,
    ))
    .parse(input)
}

/// ROS1 `time`/`duration` builtins (resolved to `builtin_interfaces` messages).
fn builtin_temporal_type(input: &str) -> IResult<&str, BaseType> {
    let (rest, kw) = alt((tag("time"), tag("duration"))).parse(input)?;
    if rest.chars().next().is_some_and(is_ident_char) {
        return Err(nom::Err::Error(nom::error::Error::new(
            input,
            nom::error::ErrorKind::Tag,
        )));
    }
    let base = match kw {
        "time" => BaseType::Time,
        "duration" => BaseType::Duration,
        _ => unreachable!(),
    };
    Ok((rest, base))
}

fn string_type(input: &str) -> IResult<&str, BaseType> {
    // `string`/`wstring` with an optional `<=N` bound. Order matters: try the
    // wide variant first so `wstring` is not misread as `w` + `string`.
    let (input, wide) =
        alt((value(true, tag("wstring")), value(false, tag("string")))).parse(input)?;
    let (input, bound) = opt(preceded(tag("<="), parse_usize)).parse(input)?;
    Ok((
        input,
        if wide {
            BaseType::WString { bound }
        } else {
            BaseType::String { bound }
        },
    ))
}

fn primitive_type(input: &str) -> IResult<&str, BaseType> {
    // Longer keywords first to avoid prefix collisions (e.g. `int16` vs `int1`).
    let (input, kw) = alt((
        tag("bool"),
        tag("byte"),
        tag("char"),
        tag("float32"),
        tag("float64"),
        tag("int8"),
        tag("uint8"),
        tag("int16"),
        tag("uint16"),
        tag("int32"),
        tag("uint32"),
        tag("int64"),
        tag("uint64"),
    ))
    .parse(input)?;

    // Reject identifiers that merely start with a keyword (`int32x`).
    if input.chars().next().is_some_and(is_ident_char) {
        return Err(nom::Err::Error(nom::error::Error::new(
            input,
            nom::error::ErrorKind::Tag,
        )));
    }

    let base = match kw {
        "bool" => BaseType::Bool,
        "byte" => BaseType::Byte,
        "char" => BaseType::Char,
        "float32" => BaseType::Float32,
        "float64" => BaseType::Float64,
        "int8" => BaseType::Int8,
        "uint8" => BaseType::Uint8,
        "int16" => BaseType::Int16,
        "uint16" => BaseType::Uint16,
        "int32" => BaseType::Int32,
        "uint32" => BaseType::Uint32,
        "int64" => BaseType::Int64,
        "uint64" => BaseType::Uint64,
        _ => unreachable!(),
    };
    Ok((input, base))
}

/// A namespaced message reference: `Name`, `pkg/Name`, or `pkg/msg/Name`.
fn named_type(input: &str) -> IResult<&str, BaseType> {
    let (input, first) = type_ident(input)?;
    let (input, rest) = opt(preceded(char('/'), type_path_tail)).parse(input)?;
    let name = match rest {
        None => TypeName {
            package: None,
            name: first,
        },
        Some((middle, last)) => {
            // `pkg/Name` -> package = pkg ; `pkg/msg/Name` -> package = pkg
            // (the conventional `msg` segment is dropped during resolution).
            let package = match middle {
                Some(m) => format!("{first}/{m}"),
                None => first,
            };
            TypeName {
                package: Some(package),
                name: last,
            }
        }
    };
    Ok((input, BaseType::Named(name)))
}

/// Parses the part after the first `/`: either `Name` or `segment/Name`.
fn type_path_tail(input: &str) -> IResult<&str, (Option<String>, String)> {
    let (input, a) = type_ident(input)?;
    let (input, b) = opt(preceded(char('/'), type_ident)).parse(input)?;
    Ok((input, b.map_or((None, a.clone()), |last| (Some(a), last))))
}

/// An array suffix: `[]`, `[N]`, or `[<=N]`.
fn array_suffix(input: &str) -> IResult<&str, ArrayKind> {
    delimited(
        char('['),
        alt((
            map(preceded(tag("<="), parse_usize), ArrayKind::Bounded),
            map(parse_usize, ArrayKind::Fixed),
            value(ArrayKind::Unbounded, space0),
        )),
        char(']'),
    )
    .parse(input)
}

/// A field/constant name (lower-snake-case by convention; we accept any
/// identifier and leave casing checks to validation).
fn identifier(input: &str) -> IResult<&str, String> {
    map(
        recognize((
            take_while1(|c: char| c.is_ascii_alphabetic() || c == '_'),
            take_while(is_ident_char),
        )),
        |s: &str| s.to_string(),
    )
    .parse(input)
}

/// A single path segment of a type name (must start with a letter).
fn type_ident(input: &str) -> IResult<&str, String> {
    map(
        recognize((
            take_while1(|c: char| c.is_ascii_alphabetic()),
            take_while(is_ident_char),
        )),
        |s: &str| s.to_string(),
    )
    .parse(input)
}

fn is_ident_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

fn parse_usize(input: &str) -> IResult<&str, usize> {
    map_res(digit1, |s: &str| s.parse::<usize>()).parse(input)
}

/// Parse a literal appropriate to `base` (and whether it is an array slot).
///
/// Returns a closure so the expected shape (bool / int / float / string /
/// array) is chosen from the declared type rather than guessed.
fn literal<'a>(
    base: &BaseType,
    is_array: bool,
) -> impl Fn(&'a str) -> IResult<&'a str, Value> + '_ {
    move |input: &'a str| {
        if is_array {
            return array_literal(base).parse(input);
        }
        scalar_literal(base).parse(input)
    }
}

fn scalar_literal<'a>(base: &BaseType) -> impl Fn(&'a str) -> IResult<&'a str, Value> + '_ {
    move |input: &'a str| match base {
        BaseType::Bool => bool_literal(input),
        BaseType::Float32 | BaseType::Float64 => map(double, Value::Float).parse(input),
        BaseType::String { .. } | BaseType::WString { .. } => {
            map(scalar_string, Value::String).parse(input)
        }
        // Numeric integer types (and `byte`/`char`).
        _ => map(integer_literal, Value::Integer).parse(input),
    }
}

fn array_element<'a>(base: &BaseType) -> impl Fn(&'a str) -> IResult<&'a str, Value> + '_ {
    move |input: &'a str| match base {
        BaseType::String { .. } | BaseType::WString { .. } => {
            map(string_literal, Value::String).parse(input)
        }
        _ => scalar_literal(base)(input),
    }
}

fn array_literal<'a>(base: &BaseType) -> impl Fn(&'a str) -> IResult<&'a str, Value> + '_ {
    move |input: &'a str| {
        let elem = array_element(base);
        map(
            delimited(
                (char('['), multispace0),
                separated_list0((multispace0, char(','), multispace0), elem),
                (multispace0, char(']')),
            ),
            Value::Array,
        )
        .parse(input)
    }
}

fn bool_literal(input: &str) -> IResult<&str, Value> {
    alt((
        value(Value::Bool(true), tag("true")),
        value(Value::Bool(false), tag("false")),
        value(Value::Bool(true), tag("1")),
        value(Value::Bool(false), tag("0")),
    ))
    .parse(input)
}

fn integer_literal(input: &str) -> IResult<&str, i128> {
    map_res(recognize((opt(one_of("+-")), digit1)), |s: &str| {
        s.parse::<i128>()
    })
    .parse(input)
}

/// A scalar string value: quoted (single/double), or — for ROS's unquoted
/// string constants/defaults — the remainder of the line, trimmed.
fn scalar_string(input: &str) -> IResult<&str, String> {
    alt((
        quoted('"'),
        quoted('\''),
        map(take_while(|_| true), |s: &str| s.trim_end().to_string()),
    ))
    .parse(input)
}

/// A string used as an array element: quoted, or a bare token delimited by
/// whitespace, comma, or the closing bracket.
fn string_literal(input: &str) -> IResult<&str, String> {
    alt((
        quoted('"'),
        quoted('\''),
        map(
            take_while1(|c: char| !c.is_whitespace() && c != ',' && c != ']'),
            |s: &str| s.to_string(),
        ),
    ))
    .parse(input)
}

fn quoted<'a>(q: char) -> impl FnMut(&'a str) -> IResult<&'a str, String> {
    move |input: &'a str| {
        let (input, _) = char(q).parse(input)?;
        let mut out = String::new();
        let mut chars = input.char_indices();
        while let Some((i, c)) = chars.next() {
            if c == '\\' {
                if let Some((_, esc)) = chars.next() {
                    out.push(match esc {
                        'n' => '\n',
                        't' => '\t',
                        'r' => '\r',
                        other => other,
                    });
                }
            } else if c == q {
                let rest = &input[i + c.len_utf8()..];
                return Ok((rest, out));
            } else {
                out.push(c);
            }
        }
        Err(nom::Err::Error(nom::error::Error::new(
            input,
            nom::error::ErrorKind::Char,
        )))
    }
}
