#!/usr/bin/env python

from test_msgs.msg import (
	Message1,
	Message2,
)


def main():
    m1 = Message1()
    print(m1)

    m2 = Message2()
    print(m2)


if __name__ == '__main__':
    main()
