import re
import os

def compress_scratchpad():
    with open('.cursor/memory/AGENT_SCRATCHPAD.md', 'r', encoding='utf-8') as f:
        content = f.read()

    # Find the sessions
    parts = re.split(r'\n(?=### \d{4}-\d{2}-\d{2})', content)
    if len(parts) <= 4:
        print(".cursor/memory/AGENT_SCRATCHPAD.md is already small enough.")
        return

    header_and_lessons = parts[0]
    sessions = parts[1:]

    # Keep last 3 sessions
    recent_sessions = sessions[-3:]
    archived_sessions = sessions[:-3]

    # Write active file
    with open('.cursor/memory/AGENT_SCRATCHPAD.md', 'w', encoding='utf-8') as f:
        f.write(header_and_lessons + "\n" + "\n".join(recent_sessions))

    # Write archive
    archive_path = '.cursor/memory/AGENT_SCRATCHPAD_ARCHIVE.md'
    mode = 'a' if os.path.exists(archive_path) else 'w'
    with open(archive_path, mode, encoding='utf-8') as f:
        if mode == 'w':
            f.write("# Agent Scratchpad Archive\n\n")
        f.write("\n".join(archived_sessions) + "\n")
    
    print(f"Archived {len(archived_sessions)} sessions from .cursor/memory/AGENT_SCRATCHPAD.md")

def compress_devlog():
    with open('.cursor/memory/DEVLOG.md', 'r', encoding='utf-8') as f:
        content = f.read()

    parts = re.split(r'\n(?=## \d{4}-\d{2}-\d{2})', content)
    if len(parts) <= 6:
        print(".cursor/memory/DEVLOG.md is already small enough.")
        return

    header = parts[0]
    sessions = parts[1:]

    # Keep last 5 sessions
    recent_sessions = sessions[-5:]
    archived_sessions = sessions[:-5]

    # Write active file
    with open('.cursor/memory/DEVLOG.md', 'w', encoding='utf-8') as f:
        f.write(header + "\n" + "\n".join(recent_sessions))

    # Write archive
    archive_path = '.cursor/memory/DEVLOG_ARCHIVE.md'
    mode = 'a' if os.path.exists(archive_path) else 'w'
    with open(archive_path, mode, encoding='utf-8') as f:
        if mode == 'w':
            f.write("# Devlog Archive\n\n")
        f.write("\n".join(archived_sessions) + "\n")
    
    print(f"Archived {len(archived_sessions)} sessions from .cursor/memory/DEVLOG.md")

if __name__ == '__main__':
    compress_scratchpad()
    compress_devlog()
