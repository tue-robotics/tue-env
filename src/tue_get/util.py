from typing import Callable, IO, Optional
import subprocess
from threading import Thread


class BackgroundPopen(subprocess.Popen):
    """
    Inspired by https://stackoverflow.com/a/42352331
    """

    @staticmethod
    def _proxy_lines(pipe: IO, handler: Callable):
        with pipe:
            for line in pipe:
                handler(line)

    def __init__(self, out_handler: Optional[Callable] = None, err_handler: Optional[Callable] = None, *args, **kwargs):
        if out_handler is not None:
            kwargs["stdout"] = subprocess.PIPE
        if err_handler is not None:
            kwargs["stderr"] = subprocess.PIPE
        super().__init__(*args, **kwargs)
        if out_handler is not None:
            Thread(target=self._proxy_lines, args=[self.stdout, out_handler]).start()
        if err_handler is not None:
            Thread(target=self._proxy_lines, args=[self.stderr, err_handler]).start()


if __name__ == "__main__":
    import shlex
    from termcolor import cprint

    cmd = """
    bash -c '
    for i in {1..100}
    do
    echo -e "${i}"
    done'
    """

    def cyan_handler(line):
        cprint(line, color="cyan", end="")

    bla = BackgroundPopen(out_handler=cyan_handler, args=shlex.split(cmd), text=True)
    bla.wait()
