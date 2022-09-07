import sys


def main():
    filename = sys.argv[1]
    with open(filename, 'r') as fp:
        lines = fp.readlines()
    lines_exit_info = [line.strip() for line in lines if "SNOPTA EXIT" in line]
    for line in lines_exit_info:
        print(line)



if __name__ == "__main__":
    main()