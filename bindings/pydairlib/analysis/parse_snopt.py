import sys


def main():
    filename = sys.argv[1]
    with open(filename, 'r') as fp:
        lines = fp.readlines()
    lines_exit_status = [line.strip() for line in lines if "SNOPTA EXIT" in line]
    lines_exit_info = [line.strip() for line in lines if "SNOPTA INFO" in line]

    for i, line in enumerate(lines_exit_status):
        if int(line.split()[2]) != 0:
            print(f'solution {i} failed, {line}, {lines_exit_info[i]}')


if __name__ == "__main__":
    main()
