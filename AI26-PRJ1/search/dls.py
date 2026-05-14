# search/dls.py
from .node import SearchNode

def dls(initial_state, limit=120):
    start_node = SearchNode(initial_state)
    if start_node.is_goal():
        return start_node.get_solution()

    # پشته شامل: (گره، عمق فعلی، مجموعه هش‌های مسیر فعلی)
    # path_hashes فقط از حلقه در شاخه جاری جلوگیری می‌کند، نه در کل درخت جستجو
    frontier = [(start_node, 0, {hash(start_node)})]

    while frontier:
        node, depth, path_hashes = frontier.pop()

        if node.is_goal():
            return node.get_solution()

        if depth < limit:
            for child in node.expand():
                child_hash = hash(child)
                if child_hash not in path_hashes:
                    # ایجاد مجموعه جدید برای شاخه فرزند (جلوگیری از تداخل شاخه‌ها)
                    new_path_hashes = path_hashes | {child_hash}
                    frontier.append((child, depth + 1, new_path_hashes))

    return []