import pandas as pd
from tqdm import tqdm


def row_to_type(type, type_args, type_kwargs):
    """
    Instantiate an object of type `type` using `row` as positional arguments and `type_kwargs` as its keyword arguments.
    :param type: The type to instantiate (e.g. TrNode)
    :param type_args: The positional arguments as an iterable
    :param type_kwargs: The keyword arguments as a dictionary (str -> Any)
    :return: The instantiated object
    """
    return type(**type_args, **type_kwargs)


def df_to_type_list(df, type, column_value_mappers=None, column_to_field_mappings=None, **type_kwargs):
    """
    Convert the Pandas DataFrame `df` to a list of instances of type `type`.
    Map column values using `column_value_mappers` first and rename columns using `column_to_field_mappings` thereafter.
    If one/both of these arguments are `None`, skip the respective action(s).
    :param df:
    :param type: The type to instantiate (e.g. TrNode)
    :param column_value_mappers: A dict of `<column_name> -> func(x)` to transform column values. `None` causes this step to be skipped.
    :param column_to_field_mappings: A dict of `<column_name> -> <field_name>` to rename columns to fit the constructor of `type`. `None` causes this step to be skipped.
    :param type_kwargs: Additional keyword arguments given to the `type` constructor for all instantiations.
    :return: The list of instances of type `type`.
    """

    # Map column values and overwrite the original column
    if column_value_mappers is not None:
        for col, mapper in column_value_mappers.items():
            df[col] = df[col].map(mapper)

    # Then rename columns (delete the one with the old name)
    if column_to_field_mappings is not None:
        for col, field in column_to_field_mappings.items():
            df[field] = df[col]
            del df[col]

    # If there is an index in the DataFrame, move its values to the `"id"` column later
    has_idx = not isinstance(df.index, pd.RangeIndex)
    ret_list = []

    #
    for row in tqdm(df.itertuples(index=has_idx), desc=f" ├─ Processing {type.__name__}s", total=len(df)):
        row_dict = row._asdict()
        if has_idx:  # Move index to `"id"` column if present
            row_dict["id"] = row.Index
            del row_dict["Index"]
        # Instantiate and append `type` object
        ret_list.append(row_to_type(type, row_dict, type_kwargs))
    return ret_list
