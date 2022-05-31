import math


def left_abbreviate(string, limit=120):
    return string if len(string) <= limit else f"...{string[:limit-3]}"


class ProgressPrinter:
    def __init__(self, verb, n) -> None:
        self.verb = verb
        self.n = n
        self.i = 0
        self.fmt_len = math.ceil(math.log10(n if n > 0 else 1))

    def step(self, msg):
        self.i += 1
        print(f"({self.i:>{self.fmt_len}d}/{self.n}) {self.verb} {left_abbreviate(msg):<120}", end="\r")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.i -= 1

        if exc_value:
            self.step("error.")
            print()
            print(exc_value)
            return
        
        self.step("done.")
        print()