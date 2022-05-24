from typing import Callable, Iterable, Optional, TypeVar


T = TypeVar("T")
def filter_none(ls: Iterable[Optional[T]]) -> filter[T]:
    return filter(lambda x: x is not None, ls)

S = TypeVar("S")
def safe_map(func: Callable[[T], S], ls: Iterable[T]) -> map[Optional[S]]:
    def safe_func(arg):
        try:
            return func(arg)
        except:
            return None
    
    return map(safe_func, ls)