from threading import Timer, Thread, Event
a = 3



class MyThread(Thread)
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

    def run(self):
        global a
        while not self.stopped.wait(5):
            global height

            print(5, 'a:', a)

stopFlag = Event()
thread = MyThread(stopFlag)
thread.start()