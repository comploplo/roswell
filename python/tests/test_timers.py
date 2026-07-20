"""asyncio timer helper: periodic callbacks that stop on cancel / node close."""

import asyncio

import roswell


def test_timer_fires_periodically():
    async def run():
        node = roswell.Node("py_timer", domain=0)
        try:
            ticks = []
            timer = node.create_timer(0.02, lambda: ticks.append(1))
            await asyncio.sleep(0.2)
            timer.cancel()
            before = len(ticks)
            await asyncio.sleep(0.1)
            # No more ticks after cancel.
            assert len(ticks) == before
            return before
        finally:
            node.close()

    fired = asyncio.run(run())
    assert fired >= 3, f"expected several ticks, got {fired}"


def test_timer_async_callback():
    async def run():
        node = roswell.Node("py_timer_async", domain=0)
        try:
            ticks = []

            async def cb():
                await asyncio.sleep(0)
                ticks.append(1)

            node.create_timer(0.02, cb)
            await asyncio.sleep(0.15)
            return len(ticks)
        finally:
            node.close()

    assert asyncio.run(run()) >= 2


def test_node_close_cancels_timers():
    async def run():
        node = roswell.Node("py_timer_close", domain=0)
        ticks = []
        node.create_timer(0.02, lambda: ticks.append(1))
        await asyncio.sleep(0.05)
        node.close()  # cancels the timer
        stopped_at = len(ticks)
        await asyncio.sleep(0.1)
        assert len(ticks) == stopped_at

    asyncio.run(run())
