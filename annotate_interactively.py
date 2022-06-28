import json
import os.path
import sys
import termcolor
from typing import Dict, List, Tuple


ignored_idfs = set()


def highlight(substr: str, text: str):
    return text.replace(substr, termcolor.colored(substr, 'green', attrs=['bold']))


def prompt_user(file: str, cb: str, idf: str, text: str) -> Tuple[str, bool, bool]:
    print(f"{file.rstrip('.cpp')}->{cb}:")
    print(highlight(idf, text))
    answer = input(f"{highlight(idf, idf)}\nwrite (w), read (r), both (rw), ignore future (i) exit and save (q), undo (z), skip (Enter): ")
    if answer not in ["", "r", "w", "rw", "q", "z", "i"]:
        print(f"Invalid answer '{answer}', try again.")
        answer = prompt_user(file, cb, idf, text)

    if answer == 'i':
        ignored_idfs.add(idf)
    elif any(x in answer for x in ['r', 'w']):
        ignored_idfs.discard(idf)

    return answer, answer == "q", answer == "z"


def main(cb_dict: Dict):
    cb_rw_dict = {}

    jobs = []

    for file, cbs in cb_dict.items():
        cb_rw_dict[file] = {}
        for cb, cb_data in cbs.items():
            cb: str
            if 'error' in cb_data:
                print(f"Error: {cb_data['error']}")
                continue
            identifiers = cb_data['identifiers']
            text_lines = cb_data['body_lines']
            text = "".join(text_lines)

            cb_rw_dict[file][cb] = {'reads': [], 'writes': []}

            for idf in identifiers:
                jobs.append((file, cb, idf, text))

    # Skip already saved mappings
    if os.path.exists("cb_mapping.json"):
        with open("cb_mapping.json", "r") as f:
            prev_rw_dict = json.load(f)
        jobs = [(file, cb, idf, text)
                for file, cb, idf, text in jobs
                if file not in prev_rw_dict
                or cb not in prev_rw_dict[file]]

    i = 0
    do_undo = False
    while i < len(jobs):
        file, cb, idf, text = jobs[i]

        if do_undo:
            ignored_idfs.discard(idf)
            cb_rw_dict[file][cb]['reads'].remove(idf)
            cb_rw_dict[file][cb]['writes'].remove(idf)
            do_undo = False

        if idf in ignored_idfs:
            i += 1
            continue

        classification, answ_quit, answ_undo = prompt_user(file, cb, idf, text)

        if answ_quit:
            del cb_rw_dict[file][cb]
            break
        elif answ_undo:
            i -= 1
            do_undo = True
            continue

        if 'r' in classification:
            cb_rw_dict[file][cb]['reads'].append(idf)
        if 'w' in classification:
            cb_rw_dict[file][cb]['writes'].append(idf)
        if not any(x in classification for x in ['r', 'w']):
            print(f"Ignoring occurences of {idf} in cb.")

        i += 1

    with open("cb_mapping.json", "w") as f:
        json.dump(cb_rw_dict, f)

    print("Done.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <cb_identifiers.json>")
        exit(0)

    with open(sys.argv[1], "r") as f:
        cb_dict = json.load(f)
        common_prefix = os.path.commonprefix(list(cb_dict.keys()))
        strip_len = len(common_prefix)
        cb_dict = {k[strip_len:]: v for k, v in cb_dict.items()}

    main(cb_dict)
