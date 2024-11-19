import asyncio


async def main():
    await asyncio.sleep(1)
    return 1


async def main2():
    await asyncio.sleep(2)
    return 2


async def main3():
    await asyncio.sleep(3)
    return 3


async def run_gathered():
    # tasks = [main(), main2(), main3()]
    # res = await asyncio.gather(*tasks)
    # return res
    task1 = asyncio.create_task(main())
    task2 = asyncio.create_task(main2())
    task3 = asyncio.create_task(main3())
    res, _ = await asyncio.wait([task1, task2, task3])
    return res


if __name__ == "__main__":
    res = asyncio.run(run_gathered())
    res = [task.result() for task in res]
    print(res)

    # loop = asyncio.get_event_loop()
    # res = loop.run_until_complete(asyncio.gather(main(), main2(), main3()))
    # print(res)
