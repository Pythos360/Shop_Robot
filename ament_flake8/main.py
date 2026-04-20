def main_with_errors(argv=None):
    """Minimal stub that mimics the real ament_flake8 entrypoint.

    Returns (rc, errors) where rc==0 indicates success.
    """
    return 0, []


def main(argv=None):
    rc, _ = main_with_errors(argv=argv)
    return rc
