# import sched, time
# s = sched.scheduler(time.time, time.sleep)
# count =0
# def do_something(sc):
#     global count
#     print("Doing stuff...")
#     count+=1
#     # do your stuff
#     if count ==10:
#         exit()
#     s.enter(1, 1, do_something, (sc,))
#
# s.enter(1, 1, do_something, (s,))
# s.run()

# import multiprocessing
#
#
# def print_records(records):
#     """
#     function to print record(tuples) in records(list)
#     """
#     for record in records:
#         print("Name: {0}\nScore: {1}\n".format(record[0], record[1]))
#
#
# def insert_record(record, records):
#     """
#     function to add a new record to records(list)
#     """
#     records.append(record)
#     print("New record added!\n")
#
#
# if __name__ == '__main__':
#     with multiprocessing.Manager() as manager:
#         # creating a list in server process memory
#         records = manager.list([('Sam', 10), ('Adam', 9), ('Kevin', 9)])
#         rec = manager.
#         # new record to be inserted in records
#         new_record = ('Jeff', 8)
#
#
#         # creating new processes
#         p1 = multiprocessing.Process(target=insert_record, args=(new_record, records))
#         p2 = multiprocessing.Process(target=print_records, args=(records,))
#
#         # running process p1 to insert new record
#         p1.start()
#         p1.join()
#
#         # running process p2 to print records
#         p2.start()
#         p2.join()


# import multiprocessing
#
#
# def square_list(mylist, q):
#     """
#     function to square a given list
#     """
#     # append squares of mylist to queue
#     for num in mylist:
#         q.put(num * num)
#
#
# def print_queue(q):
#     """
#     function to print queue elements
#     """
#     print("Queue elements:")
#     while not q.empty():
#         print(q.get())
#     print("Queue is now empty!")
#
#
# if __name__ == "__main__":
#     # input list
#     mylist = [1, 2, 3, 4]
#
#     # creating multiprocessing Queue
#     q = multiprocessing.Queue()
#
#     # creating new processes
#     p1 = multiprocessing.Process(target=square_list, args=(mylist, q))
#     p2 = multiprocessing.Process(target=print_queue, args=(q,))
#
#     # running process p1 to square list
#     p1.start()
#     p1.join()
#
#     # running process p2 to get queue elements
#     p2.start()
#     p2.join()


# from multiprocessing import Process, Manager
# from multiprocessing.managers import BaseManager
#
# class GetSetter():
#     def __init__(self):
#         self.var = None
#
#     def set(self, value):
#         self.var = value
#
#     def get(self):
#         return self.var
#
#
# class ChildClass(GetSetter):
#     pass
#
# class ParentClass(GetSetter):
#     def __init__(self):
#         self.child = ChildClass()
#         GetSetter.__init__(self)
#
#     def getChild(self):
#         return self.child
#
#
# def change_obj_value(obj):
#     obj.set(100)
#     obj.getChild().set(100)
#
#
# if __name__ == '__main__':
#     BaseManager.register('ParentClass', ParentClass)
#     manager = BaseManager()
#     manager.start()
#     inst2 = manager.ParentClass()
#
#     p2 = Process(target=change_obj_value, args=[inst2])
#     p2.start()
#     p2.join()
#
#     print(inst2)                    # <__main__.ParentClass object at 0x10cf82350>
#     print(inst2.getChild())         # <__main__.ChildClass object at 0x10cf6dc50>
#     print(inst2.get())              # 100
#     #good!
#
#     print(inst2.getChild().get())   # None
#     #bad! you need to register child class too but there's almost no way to do it
#     #even if you did register child class, you may get PicklingError :)


# import multiprocessing
# import time
#
#
# def sender(conn, msgs):
#     """
#     function to send messages to other end of pipe
#     """
#     for msg in msgs:
#         conn.send(msg)
#         print("Sent the message: {}".format(msg))
#         time.sleep(1)
#
#     conn.close()
#
#
# def receiver(conn):
#     """
#     function to print the messages received from other
#     end of pipe
#     """
#     while 1:
#         msg = conn.recv()
#         if msg == "problem":
#             break
#         print("Received the message: {} \n".format(msg))
#
#
# if __name__ == "__main__":
#     # messages to be sent
#     msgs = ["hello", "hey", "hru?", "END", "this", "might", "solve", "my", "problem"]
#
#     # creating a pipe
#     parent_conn, child_conn = multiprocessing.Pipe()
#
#     # creating new processes
#     p1 = multiprocessing.Process(target=sender, args=(parent_conn, msgs))
#     p2 = multiprocessing.Process(target=receiver, args=(child_conn,))
#
#     # running processes
#     p1.start()
#     p2.start()
#
#     # wait until processes finish
#     p1.join()
#     p2.join()

import math

# def create_robot(length0,length1,min0,max0,maxvel0,min1,max1,maxvel1):
#     print("initilized with following values: \n")
#     print(f"robot link0 length0: {length0}, min0: {min0}, max0: {max0},maxvel0: {maxvel0} ")
#     print(f"robot link1 length0: {length1}, min1: {min1}, max1: {max1},maxvel1: {maxvel1} ")
#
#     pass
#
# def read_description(filename, robot=True,path = False, sep = ' '):
#     if robot==True and path == False:
#         f = open(filename, "r")
#         fields = f.readline().split(sep=sep)
#         print(f"number of variables: {len(fields)}")
#         if len(fields) == 8:
#             print("initializing robot with description...")
#             A = [float(x) for x in fields]
#             print(f"A: {A}")
#             length0, length1, min0, max0, maxvel0, min1, max1, maxvel1 = A
#             print(f"robot link0 length0: {length0}, min0: {min0}, max0: {max0},maxvel0: {maxvel0} ")
#             print(f"robot link1 length0: {length1}, min1: {min1}, max1: {max1},maxvel1: {maxvel1} ")
#
#             print(f"angle in radians> {round(math.radians(min0),6)}, {math.radians(max0)}")
#         else:
#             pass

# read_description('robot_description.txt',robot=True, path= False)


# def read_path_description(filename, sep = ' '):
#     f = open(filename, 'r')
#     fields = f.readlines()
#
#     store_lines = []
#     print(len(fields))
#     for line in fields:
#         fields = line.split(sep)
#         print(fields)
#         try:
#             fields_as_float = [float(x) for x in fields]
#             store_lines.append(fields_as_float)
#             print(f"fields_as_float: {fields_as_float}")
#         except:
#             print("error: int or float expected...")
#             return []
#     print(f"firsr line {store_lines[0]} and last line {store_lines[-1]}")
#
# read_path_description('path_point.txt')

# 1.5 1.0 0.0 1.5 0.0 0.0
# 1.5 0.0 0.0 2.0 0.0 0.0
# 2.0 0.0 0.0 2.0 1.0 0.0
# 2.0 1.0 0.0 1.5 1.0 0.0

import multiprocessing
import time
import os
os.system("")


# def hang():
#     while True:
#         print ('hanging..')
#         time.sleep(10)

def count(conn):
    curr_count = 0
    try:

        for i in range(10):
            curr_count+=1
            conn.send(curr_count)
            print("waiting")
            time.sleep(1)
    except KeyboardInterrupt:
        print("stopping the count")
        return

def print_count(conn):
    try:
        while True:
            msg = conn.recv()
            print(f"current count {msg}")
    except KeyboardInterrupt:
        print("interruption received: stopping the count")
        return


def main():

    parent, child = multiprocessing.Pipe()

    p1 = multiprocessing.Process(target=count, args=(parent,))
    p2 = multiprocessing.Process(target=print_count, args=(child,))
    try:

        p1.start()
        p2.start()

        p1.join()
        p2.join()

    except KeyboardInterrupt:
        print("stopping all the process")
        if p1.is_alive():
            p1.terminate()
        if p2.is_alive():
            p2.terminate()

        print("all the process stopped")

    # try:
    #     p.start()
    #     time.sleep(10)
    #     p.terminate()
    #     p.join()
    #     print ('main process exiting..')
    # except KeyboardInterrupt:
    #     if p.is_alive():
    #         p.terminate()
    #         # print("killed process")
    #         CRED = '\033[91m'
    #         CEND = '\033[0m'
    #         print(CRED + "Error, does not compute!" + CEND)


if __name__ == '__main__':
    main()