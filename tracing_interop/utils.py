import sys

import pandas as pd


def row_to_type(row, type, has_idx, **type_kwargs):
    return type(id=row.name, **row, **type_kwargs) if has_idx else type(**row, **type_kwargs)


def df_to_type_list(df, type, **type_kwargs):
    has_idx = not isinstance(df.index, pd.RangeIndex)
    return [row_to_type(row, type, has_idx, **type_kwargs) for _, row in df.iterrows()]


def by_index(df, index, type):
    return df_to_type_list(df.loc[index], type)


def by_column(df, column_name, column_val, type):
    return df_to_type_list(df[df[column_name] == column_val], type)


def list_to_dict(ls, key='id'):
    return {getattr(item, key): item for item in ls}
