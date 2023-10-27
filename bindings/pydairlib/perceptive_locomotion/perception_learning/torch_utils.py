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


def main() -> None:
    """
    Tests what device is returned by get_device(). The output should look
    something like:
                       tensor([1.], device='cuda:0')

    If no device is shown, you are using the CPU
    """
    device = get_device()
    x = torch.ones(1, device=device)
    print(x)


if __name__ == '__main__':
    main()
