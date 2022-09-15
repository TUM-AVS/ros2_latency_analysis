import pandas as pd
from tqdm import tqdm


def row_to_type(row, type, **type_kwargs):
    return type(**row, **type_kwargs)


def df_to_type_list(df, type, mappers=None, **type_kwargs):
    if mappers is not None:
        for col, mapper in mappers.items():
            df[col] = df[col].map(mapper)

    has_idx = not isinstance(df.index, pd.RangeIndex)
    ret_list = []

    for row in tqdm(df.itertuples(index=has_idx), desc=f" ├─ Processing {type.__name__}s", total=len(df)):
        row_dict = row._asdict()
        if has_idx:
            row_dict["id"] = row.Index
            del row_dict["Index"]
        ret_list.append(row_to_type(row_dict, type, **type_kwargs))
    return ret_list
