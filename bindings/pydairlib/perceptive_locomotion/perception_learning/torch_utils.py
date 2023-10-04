import torch


def get_device():
    """
        :return: Returns the highest performance device available in the current
        pytorch configuration
    """

    if torch.cuda.is_available():
        return torch.device('cuda')

    if torch.backends.mps.is_available():
        return torch.device('mps')

    return torch.device('cpu')


def main():
    device = get_device()
    x = torch.ones(1, device=device)
    print(x)


if __name__ == '__main__':
    main()
