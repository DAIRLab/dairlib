"""Create entry point for the command-line interface (CLI)."""

import argparse
import importlib
from importlib.metadata import PackageNotFoundError
import sys
from typing import Any, Dict, List, Optional

import scooby
from scooby.report import Report, get_distribution_dependencies


def main(args: Optional[List[str]] = None):
    """Parse command line inputs of CLI interface."""
    # If not explicitly called, catch arguments
    if args is None:
        args = sys.argv[1:]

    # Start CLI-arg-parser and define arguments
    parser = argparse.ArgumentParser(description="Great Dane turned Python environment detective.")

    # arg: Packages
    parser.add_argument(
        "packages", nargs="*", default=None, type=str, help=("names of the packages to report")
    )

    # arg: Report of a package
    parser.add_argument(
        "--report", "-r", default=None, type=str, help=("print `Report()` of this package")
    )

    # arg: Sort
    parser.add_argument(
        "--no-opt",
        action="store_true",
        default=None,
        help="do not show the default optional packages. Defaults to True if using --report and defaults to False otherwise.",
    )

    # arg: Sort
    parser.add_argument(
        "--sort",
        action="store_true",
        default=False,
        help="sort the packages when the report is shown",
    )

    # arg: Version
    parser.add_argument(
        "--version", "-v", action="store_true", default=False, help="only display scooby version"
    )

    # Call act with command line arguments as dict.
    act(vars(parser.parse_args(args)))


def act(args_dict: Dict[str, Any]) -> None:
    """Act upon CLI inputs."""
    # Quick exit if only scooby version.
    if args_dict.pop('version'):
        print(f"scooby v{scooby.__version__}")
        return

    report = args_dict.pop('report')
    no_opt = args_dict.pop('no_opt')
    packages = args_dict.pop('packages')

    if no_opt is None:
        if report is None:
            no_opt = False
        else:
            no_opt = True

    # Report of another package.
    if report:
        try:
            module = importlib.import_module(report)
        except ImportError:
            pass
        else:
            try:
                print(module.Report())
                return
            except AttributeError:
                pass

        try:
            dist_deps = get_distribution_dependencies(report)
            packages = [report, *dist_deps, *packages]
        except PackageNotFoundError:
            print(
                f"Package `{report}` has no Report class and `importlib` could not be used to autogenerate one.",
                file=sys.stderr,
            )
            sys.exit(1)

    # Collect input.
    inp = {'additional': packages, 'sort': args_dict['sort']}

    # Define optional as empty list if no-opt.
    if no_opt:
        inp['optional'] = []

    # Print the report.
    print(Report(**inp))


if __name__ == "__main__":
    main()
