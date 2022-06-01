#!/usr/bin/env python3.8
"""
Test 4: Combining the ThreadPoolExecutor
with the asyncio package.
"""
import time
from concurrent.futures import ThreadPoolExecutor
import asyncio


def vThreadFunction():
    """Function to do CPU-bound work.
    Args:
    Returns:
    """
    iResult = 0
    for iCnt in range(1000000):
        iResult += iCnt


async def vMain():
    loop = asyncio.get_running_loop()

    lstFutures = []

    # Create an executor with a maximum of eight workers
    objExecutor = ThreadPoolExecutor(max_workers=8)

    fTimePrefCountStart = time.perf_counter()

    # Create eight threads using the executor
    for _ in range(1):
        lstFutures.append(loop.run_in_executor(objExecutor, vThreadFunction))

    # Wait for all threads to complete
    await asyncio.wait(lstFutures)

    fTimePrefCountEnd = time.perf_counter()
    print(f"Delta time {fTimePrefCountEnd - fTimePrefCountStart} [s]")

    objExecutor.shutdown(wait=False)


if __name__ == "__main__":
    # Python 3.7+
    asyncio.run(vMain())