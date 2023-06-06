from threading import Thread, Event
import time


def light():
    print('红灯正亮着')
    time.sleep(0.1)
    event.set()  # 绿灯亮
    print("绿灯亮")
    event.clear()


def car(name):
    print('车%s正在等绿灯' % name)
    event.wait()  # 等灯绿 此时event为False,直到event.set()将其值设置为True,才会继续运行.
    print('车%s通行' % name)


if __name__ == '__main__':
    event = Event()
    print(event.is_set())

    # 红绿灯
    t1 = Thread(target=light)
    t1.start()
    # 车
    for i in range(10):
        t = Thread(target=car, args=(i,))
        t.start()
