//! Parser for the OMG IDL subset that `rosidl` emits.
//!
//! Supports nested `module`s, `struct`s (-> messages), `const`s (carried in the
//! conventional `<Name>_Constants` module), primitive/scoped/`string`/`wstring`
//! types, `sequence<T[, N]>`, fixed arrays `T name[N]`, and `@annotations`
//! (skipped). Produces the same `(MsgId, MessageSpec)` inputs the `.msg`
//! frontend feeds into [`crate::resolve`].

use nom::{
    IResult, Parser,
    branch::alt,
    bytes::complete::{tag, take_while, take_while1},
    character::complete::{char, digit1, multispace0, one_of},
    combinator::{map, map_res, opt, recognize, value},
    multi::{many0, separated_list1},
    number::complete::double,
    sequence::{delimited, preceded},
};

use crate::ast::*;
use crate::ir::MsgId;
use crate::parser::ParseError;

/// Parse an `.idl` file into zero or more messages.
pub fn parse_idl(input: &str) -> Result<Vec<(MsgId, MessageSpec)>, ParseError> {
    let cleaned = strip_comments(input);
    let items = match delimited(multispace0, many0(item), multispace0).parse(&cleaned) {
        Ok((rest, items)) if rest.trim().is_empty() => items,
        Ok((rest, _)) => {
            return Err(ParseError {
                line: 0,
                content: rest.chars().take(40).collect(),
                message: "unexpected IDL near here".to_string(),
            });
        }
        Err(e) => {
            return Err(ParseError {
                line: 0,
                content: String::new(),
                message: format!("IDL parse error: {e}"),
            });
        }
    };

    // Flatten the module tree: collect structs as messages, then attach
    // constants from sibling `<Name>_Constants` modules.
    let mut messages: Vec<(MsgId, MessageSpec)> = Vec::new();
    let mut consts: Vec<(String, String, Vec<Constant>)> = Vec::new(); // (pkg, base, consts)
    flatten(&items, &[], &mut messages, &mut consts);

    for (pkg, base, cs) in consts {
        if let Some((_, spec)) = messages
            .iter_mut()
            .find(|(id, _)| id.package == pkg && id.name == base)
        {
            spec.constants = cs;
        }
    }
    Ok(messages)
}

#[derive(Debug)]
enum Item {
    Module { name: String, items: Vec<Item> },
    Struct { name: String, fields: Vec<Field> },
    Const(Constant),
}

/// The package for a module path: segments minus the `msg`/`srv`/`action`
/// subnamespace, joined by `/`.
fn package_of(path: &[String]) -> String {
    path.iter()
        .filter(|s| !matches!(s.as_str(), "msg" | "srv" | "action"))
        .cloned()
        .collect::<Vec<_>>()
        .join("/")
}

fn flatten(
    items: &[Item],
    path: &[String],
    messages: &mut Vec<(MsgId, MessageSpec)>,
    consts: &mut Vec<(String, String, Vec<Constant>)>,
) {
    let pkg = package_of(path);
    for it in items {
        match it {
            Item::Struct { name, fields } => messages.push((
                MsgId::new(pkg.clone(), name.clone()),
                MessageSpec {
                    constants: Vec::new(),
                    fields: fields.clone(),
                },
            )),
            Item::Module { name, items } => {
                if let Some(base) = name.strip_suffix("_Constants") {
                    let cs: Vec<Constant> = items
                        .iter()
                        .filter_map(|i| match i {
                            Item::Const(c) => Some(c.clone()),
                            _ => None,
                        })
                        .collect();
                    consts.push((pkg.clone(), base.to_string(), cs));
                } else {
                    let mut next = path.to_vec();
                    next.push(name.clone());
                    flatten(items, &next, messages, consts);
                }
            }
            Item::Const(_) => {}
        }
    }
}

// ---- grammar ------------------------------------------------------------

fn item(input: &str) -> IResult<&str, Item> {
    let (input, ()) = annotations(input)?;
    alt((module, structure, const_decl)).parse(input)
}

fn module(input: &str) -> IResult<&str, Item> {
    let (input, _) = kw("module")(input)?;
    let (input, name) = ws(identifier).parse(input)?;
    let (input, items) = delimited(sym("{"), many0(item), sym("}")).parse(input)?;
    let (input, _) = sym(";")(input)?;
    Ok((input, Item::Module { name, items }))
}

fn structure(input: &str) -> IResult<&str, Item> {
    let (input, _) = kw("struct")(input)?;
    let (input, name) = ws(identifier).parse(input)?;
    let (input, fields) = delimited(sym("{"), many0(member), sym("}")).parse(input)?;
    let (input, _) = sym(";")(input)?;
    Ok((
        input,
        Item::Struct {
            name,
            fields: fields.into_iter().flatten().collect(),
        },
    ))
}

/// A struct member: `[annotations] type decl[, decl]* ;` — may declare several
/// fields sharing one type.
fn member(input: &str) -> IResult<&str, Vec<Field>> {
    let (input, ()) = annotations(input)?;
    let (input, (base, seq_array)) = type_spec(input)?;
    let (input, decls) = separated_list1(sym(","), declarator).parse(input)?;
    let (input, _) = sym(";")(input)?;
    let fields = decls
        .into_iter()
        .map(|(name, dim)| Field {
            ty: FieldType {
                base: base.clone(),
                array: seq_array.or(dim),
            },
            name,
            default: None,
        })
        .collect();
    Ok((input, fields))
}

/// `name` or `name[N]`.
fn declarator(input: &str) -> IResult<&str, (String, Option<ArrayKind>)> {
    let (input, name) = ws(identifier).parse(input)?;
    let (input, dim) = opt(delimited(sym("["), ws(usize_lit), sym("]"))).parse(input)?;
    Ok((input, (name, dim.map(ArrayKind::Fixed))))
}

/// Returns the element base type plus an array kind if it is a `sequence<>`.
fn type_spec(input: &str) -> IResult<&str, (BaseType, Option<ArrayKind>)> {
    alt((sequence_type, map(scalar_type, |b| (b, None)))).parse(input)
}

fn sequence_type(input: &str) -> IResult<&str, (BaseType, Option<ArrayKind>)> {
    let (input, _) = kw("sequence")(input)?;
    let (input, _) = sym("<")(input)?;
    let (input, base) = scalar_type(input)?;
    let (input, bound) = opt(preceded(sym(","), ws(usize_lit))).parse(input)?;
    let (input, _) = sym(">")(input)?;
    let array = Some(bound.map_or(ArrayKind::Unbounded, ArrayKind::Bounded));
    Ok((input, (base, array)))
}

fn scalar_type(input: &str) -> IResult<&str, BaseType> {
    ws(alt((string_type, primitive_type, scoped_type))).parse(input)
}

fn string_type(input: &str) -> IResult<&str, BaseType> {
    let (input, wide) =
        alt((value(true, tag("wstring")), value(false, tag("string")))).parse(input)?;
    let (input, bound) = opt(delimited(sym("<"), ws(usize_lit), sym(">"))).parse(input)?;
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
    // Longest spellings first. Covers IDL traditional and explicit forms.
    let (input, base) = alt((
        value(BaseType::Uint64, tag("unsigned long long")),
        value(BaseType::Int64, tag("long long")),
        value(BaseType::Uint32, tag("unsigned long")),
        value(BaseType::Uint16, tag("unsigned short")),
        value(BaseType::Float64, tag("double")),
        value(BaseType::Float32, tag("float")),
        value(BaseType::Int16, tag("short")),
        value(BaseType::Int32, tag("long")),
        value(BaseType::Bool, tag("boolean")),
        value(BaseType::Byte, tag("octet")),
        value(BaseType::Char, tag("char")),
        // Explicit width spellings (IDL 4.2 / rosidl).
        value(BaseType::Uint8, tag("uint8")),
        value(BaseType::Int8, tag("int8")),
        value(BaseType::Uint16, tag("uint16")),
        value(BaseType::Int16, tag("int16")),
        value(BaseType::Uint32, tag("uint32")),
        value(BaseType::Int32, tag("int32")),
        value(BaseType::Uint64, tag("uint64")),
        value(BaseType::Int64, tag("int64")),
    ))
    .parse(input)?;
    // Reject when it is actually a longer identifier (e.g. `int8_helper`).
    if input.chars().next().is_some_and(is_ident_char) {
        return Err(nom::Err::Error(nom::error::Error::new(
            input,
            nom::error::ErrorKind::Tag,
        )));
    }
    Ok((input, base))
}

/// A scoped name `A::B::C` -> `TypeName { package: "A/B", name: "C" }`.
fn scoped_type(input: &str) -> IResult<&str, BaseType> {
    let (input, segs) = separated_list1(tag("::"), identifier).parse(input)?;
    let (last, pkg) = segs.split_last().unwrap();
    let package = pkg
        .iter()
        .filter(|s| !matches!(s.as_str(), "msg" | "srv" | "action"))
        .cloned()
        .collect::<Vec<_>>()
        .join("/");
    let name = TypeName {
        package: if package.is_empty() {
            None
        } else {
            Some(package)
        },
        name: last.clone(),
    };
    Ok((input, BaseType::Named(name)))
}

fn const_decl(input: &str) -> IResult<&str, Item> {
    let (input, _) = kw("const")(input)?;
    let (input, base) = scalar_type(input)?;
    let (input, name) = ws(identifier).parse(input)?;
    let (input, _) = sym("=")(input)?;
    let (input, val) = ws(|i| const_value(&base, i)).parse(input)?;
    let (input, _) = sym(";")(input)?;
    Ok((
        input,
        Item::Const(Constant {
            ty: base,
            name,
            value: val,
        }),
    ))
}

fn const_value<'a>(base: &BaseType, input: &'a str) -> IResult<&'a str, Value> {
    match base {
        BaseType::Bool => alt((
            value(Value::Bool(true), alt((tag("TRUE"), tag("true")))),
            value(Value::Bool(false), alt((tag("FALSE"), tag("false")))),
        ))
        .parse(input),
        BaseType::Float32 | BaseType::Float64 => map(double, Value::Float).parse(input),
        BaseType::String { .. } | BaseType::WString { .. } => {
            map(string_lit, Value::String).parse(input)
        }
        _ => map(
            map_res(recognize((opt(one_of("+-")), digit1)), |s: &str| {
                s.parse::<i128>()
            }),
            Value::Integer,
        )
        .parse(input),
    }
}

fn string_lit(input: &str) -> IResult<&str, String> {
    delimited(
        char('"'),
        map(take_while(|c| c != '"'), |s: &str| s.to_string()),
        char('"'),
    )
    .parse(input)
}

/// Zero or more `@name` / `@name(...)` annotations (ignored).
fn annotations(input: &str) -> IResult<&str, ()> {
    let (input, _) = multispace0(input)?;
    let (input, _) = many0(annotation).parse(input)?;
    Ok((input, ()))
}

fn annotation(input: &str) -> IResult<&str, ()> {
    let (input, _) = char('@').parse(input)?;
    let (input, _) = identifier(input)?;
    // The argument list may be separated from the name by whitespace.
    let (input, _) = opt(preceded(
        multispace0,
        delimited(char('('), take_while(|c| c != ')'), char(')')),
    ))
    .parse(input)?;
    let (input, _) = multispace0(input)?;
    Ok((input, ()))
}

// ---- lexical helpers ----------------------------------------------------

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

fn is_ident_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

fn usize_lit(input: &str) -> IResult<&str, usize> {
    map_res(digit1, str::parse::<usize>).parse(input)
}

/// Wrap a parser to consume surrounding whitespace.
fn ws<'a, F, O>(inner: F) -> impl Parser<&'a str, Output = O, Error = nom::error::Error<&'a str>>
where
    F: Parser<&'a str, Output = O, Error = nom::error::Error<&'a str>>,
{
    delimited(multispace0, inner, multispace0)
}

/// A punctuation symbol surrounded by optional whitespace.
fn sym<'a>(s: &'static str) -> impl FnMut(&'a str) -> IResult<&'a str, &'a str> {
    move |i| delimited(multispace0, tag(s), multispace0).parse(i)
}

/// A keyword: matched only when not followed by an identifier character.
fn kw<'a>(s: &'static str) -> impl FnMut(&'a str) -> IResult<&'a str, &'a str> {
    move |i| {
        let (i, _) = multispace0(i)?;
        let (i, m) = tag(s)(i)?;
        if i.chars().next().is_some_and(is_ident_char) {
            return Err(nom::Err::Error(nom::error::Error::new(
                i,
                nom::error::ErrorKind::Tag,
            )));
        }
        let (i, _) = multispace0(i)?;
        Ok((i, m))
    }
}

/// Remove `//` and `/* */` comments, preserving string literals.
fn strip_comments(src: &str) -> String {
    let b = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0;
    let mut in_str = false;
    while i < b.len() {
        let c = b[i] as char;
        if in_str {
            out.push(c);
            if c == '"' {
                in_str = false;
            }
            i += 1;
        } else if c == '"' {
            in_str = true;
            out.push(c);
            i += 1;
        } else if c == '/' && i + 1 < b.len() && b[i + 1] == b'/' {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
        } else if c == '/' && i + 1 < b.len() && b[i + 1] == b'*' {
            i += 2;
            while i + 1 < b.len() && !(b[i] == b'*' && b[i + 1] == b'/') {
                i += 1;
            }
            i += 2;
        } else {
            out.push(c);
            i += 1;
        }
    }
    out
}
