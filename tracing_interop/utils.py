import sys

import pandas as pd
from tqdm.notebook import tqdm


def row_to_type(row, type, **type_kwargs):
    return type(**row, **type_kwargs)


def df_to_type_list(df, type, **type_kwargs):
    has_idx = not isinstance(df.index, pd.RangeIndex)
    ret_list = []
    p = tqdm(desc=" ├─ Processing", total=len(df))
    for row in df.itertuples(index=has_idx):
        p.update()
        row_dict = row._asdict()
        if has_idx:
            row_dict["id"] = row.Index
            del row_dict["Index"]
        ret_list.append(row_to_type(row_dict, type, **type_kwargs))
    return ret_list


def by_index(df, index, type):
    return df_to_type_list(df.loc[index], type)


def by_column(df, column_name, column_val, type):
    return df_to_type_list(df[df[column_name] == column_val], type)


def list_to_dict(ls, key='id'):
    return {getattr(item, key): item for item in ls}
