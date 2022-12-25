import pandas as pd
from tqdm import tqdm


def row_to_type(row, type, **type_kwargs):
    return type(**row, **type_kwargs)


def df_to_type_list(df, type, column_value_mappers=None, column_to_field_mappings=None, **type_kwargs):
    if column_value_mappers is not None:
        for col, mapper in column_value_mappers.items():
            df[col] = df[col].map(mapper)

    if column_to_field_mappings is not None:
        for col, field in column_to_field_mappings.items():
            df[field] = df[col]
            del df[col]

    has_idx = not isinstance(df.index, pd.RangeIndex)
    ret_list = []

    for row in tqdm(df.itertuples(index=has_idx), desc=f" ├─ Processing {type.__name__}s", total=len(df)):
        row_dict = row._asdict()
        if has_idx:
            row_dict["id"] = row.Index
            del row_dict["Index"]
        ret_list.append(row_to_type(row_dict, type, **type_kwargs))
    return ret_list
