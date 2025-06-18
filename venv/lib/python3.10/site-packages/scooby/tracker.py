"""Track imports."""

from types import ModuleType
from typing import List, Mapping, Optional, Sequence, Set, Union

from scooby.knowledge import get_standard_lib_modules
from scooby.report import Report

TRACKING_SUPPORTED = False
SUPPORT_MESSAGE = (
    "Tracking is not supported for this version of Python. " "Try using a modern version of Python."
)
try:
    import builtins

    CLASSIC_IMPORT = builtins.__import__
    TRACKING_SUPPORTED = True
except (ImportError, AttributeError):
    pass

# The variable we track all imports in
TRACKED_IMPORTS: List[Union[str, ModuleType]] = ["scooby"]

MODULES_TO_IGNORE = {
    "pyMKL",
    "mkl",
    "vtkmodules",
    "mpl_toolkits",
}


STDLIB_PKGS: Optional[Set[str]] = None


def _criterion(name: str):
    if (
        len(name) > 0
        and name not in STDLIB_PKGS
        and not name.startswith("_")
        and name not in MODULES_TO_IGNORE
    ):
        return True
    return False


if TRACKING_SUPPORTED:

    def scooby_import(
        name: str,
        globals: Optional[Mapping[str, object]] = None,
        locals: Optional[Mapping[str, object]] = None,
        fromlist: Sequence[str] = (),
        level: int = 0,
    ) -> ModuleType:
        """Override of the import method to track package names."""
        m = CLASSIC_IMPORT(name, globals=globals, locals=locals, fromlist=fromlist, level=level)
        name = name.split(".")[0]
        if level == 0 and _criterion(name):
            TRACKED_IMPORTS.append(name)
        return m


def track_imports() -> None:
    """Track all imported modules for the remainder of this session."""
    if not TRACKING_SUPPORTED:
        raise RuntimeError(SUPPORT_MESSAGE)
    global STDLIB_PKGS
    STDLIB_PKGS = get_standard_lib_modules()
    builtins.__import__ = scooby_import
    return


def untrack_imports() -> None:
    """Stop tracking imports and return to the builtin import method.

    This will also clear the tracked imports.
    """
    if not TRACKING_SUPPORTED:
        raise RuntimeError(SUPPORT_MESSAGE)
    builtins.__import__ = CLASSIC_IMPORT
    TRACKED_IMPORTS.clear()
    TRACKED_IMPORTS.append("scooby")
    return


class TrackedReport(Report):
    """A class to inspect the active environment and generate a report.

    Generates a report based on all imported modules. Simply pass the
    ``globals()`` dictionary.
    """

    def __init__(
        self,
        additional: Optional[List[Union[str, ModuleType]]] = None,
        ncol: int = 3,
        text_width: int = 80,
        sort: bool = False,
    ):
        """Initialize."""
        if not TRACKING_SUPPORTED:
            raise RuntimeError(SUPPORT_MESSAGE)
        if len(TRACKED_IMPORTS) < 2:
            raise RuntimeError(
                "There are no tracked imports, please use "
                "`scooby.track_imports()` before running your "
                "code."
            )

        Report.__init__(
            self,
            additional=additional,
            core=TRACKED_IMPORTS,
            ncol=ncol,
            text_width=text_width,
            sort=sort,
            optional=[],
        )
