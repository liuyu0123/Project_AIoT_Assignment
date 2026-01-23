#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import shutil
import sqlite3
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Merge ROS2 bags (sqlite3 backend)"
    )
    parser.add_argument("--base", required=True, help="Base bag directory")
    parser.add_argument("--replay", required=True, help="Replay bag directory")
    parser.add_argument("--out", required=True, help="Output merged bag directory")
    parser.add_argument(
        "--override-topics",
        nargs="*",
        default=[],
        help="Topics to override using replay bag"
    )
    return parser.parse_args()


def find_db(bag_dir: Path):
    for f in bag_dir.iterdir():
        if f.suffix == ".db3":
            return f
    raise RuntimeError(f"No .db3 found in {bag_dir}")


def copy_base_bag(base: Path, out: Path):
    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(base, out)


def delete_topics(db_path: Path, topics):
    if not topics:
        return

    conn = sqlite3.connect(str(db_path))
    cur = conn.cursor()

    placeholders = ",".join("?" for _ in topics)
    cur.execute(
        f"SELECT id FROM topics WHERE name IN ({placeholders})",
        topics
    )
    topic_ids = [r[0] for r in cur.fetchall()]

    if not topic_ids:
        conn.close()
        return

    id_placeholders = ",".join("?" for _ in topic_ids)

    cur.execute(
        f"DELETE FROM messages WHERE topic_id IN ({id_placeholders})",
        topic_ids
    )
    cur.execute(
        f"DELETE FROM topics WHERE id IN ({id_placeholders})",
        topic_ids
    )

    conn.commit()
    conn.close()


def insert_replay_data(base_db: Path, replay_db: Path):
    base = sqlite3.connect(str(base_db))
    replay = sqlite3.connect(str(replay_db))

    cb = base.cursor()
    cr = replay.cursor()

    cr.execute(
        "SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics"
    )
    replay_topics = cr.fetchall()

    topic_id_map = {}

    for _, name, ttype, fmt, qos in replay_topics:
        cb.execute(
            "INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) "
            "VALUES (?, ?, ?, ?)",
            (name, ttype, fmt, qos)
        )
        topic_id_map[name] = cb.lastrowid

    cr.execute("SELECT topic_id, timestamp, data FROM messages")
    for old_tid, ts, data in cr.fetchall():
        cr.execute("SELECT name FROM topics WHERE id=?", (old_tid,))
        name = cr.fetchone()[0]
        cb.execute(
            "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
            (topic_id_map[name], ts, data)
        )

    base.commit()
    base.close()
    replay.close()


def main():
    args = parse_args()

    base = Path(args.base).resolve()
    replay = Path(args.replay).resolve()
    out = Path(args.out).resolve()

    print(f"[merge] base    : {base}")
    print(f"[merge] replay  : {replay}")
    print(f"[merge] output  : {out}")
    print(f"[merge] override topics: {args.override_topics}")

    copy_base_bag(base, out)

    base_db = find_db(out)
    replay_db = find_db(replay)

    delete_topics(base_db, args.override_topics)
    insert_replay_data(base_db, replay_db)

    print("[merge] done")


if __name__ == "__main__":
    main()
