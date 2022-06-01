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
    lstProcesses = []

    # Create eight processes
    for _ in range(8):
        lstProcesses.append(Process(target=vProcessFunction))

    fTimePrefCountStart = time.perf_counter()

    # Start all the processes
    for objProcess in lstProcesses:
        objProcess.start()

    # Wait for all processes to complete
    for objProcess in lstProcesses:
        objProcess.join()

    fTimePrefCountEnd = time.perf_counter()
    print(f"Delta time {fTimePrefCountEnd - fTimePrefCountStart} [s]")


if __name__ == "__main__":
    vMain()