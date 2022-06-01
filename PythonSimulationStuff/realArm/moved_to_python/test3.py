#!/usr/bin/env python3.8
"""
Test 5: Making use of the multiprocessing package.
"""
import time
from multiprocessing import Process


def vProcessFunction():
    """Function to do CPU-bound work.
    Args:
    Returns:
    """
    iResult = 0
    for iCnt in range(1000000):
        iResult += iCnt


def vMain():
    fTimePrefCountStart = time.perf_counter()

    vProcessFunction()

    fTimePrefCountEnd = time.perf_counter()
    print(f"Delta time {fTimePrefCountEnd - fTimePrefCountStart} [s]")


if __name__ == "__main__":
    vMain()