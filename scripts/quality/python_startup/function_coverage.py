"""Opt-in Python *entry* coverage, independent of executing a ``def`` line.

Code-constant paths distinguish same-line callable bodies where the compiler
retains separate code objects. Compiler-coalesced aliases remain conservatively
uncovered in the static AST denominator, including definitions never imported.
Class/module bodies and compiler-generated comprehensions are not functions.
"""
import ast
import atexit
from collections import Counter
import json
import os
from pathlib import Path
import sys
import threading
from types import CodeType
import uuid


def code_functions(code):
    """Return stable identities for declared functions, methods and lambdas."""
    result = []

    def visit(parent, path=()):
        if (parent.co_flags & 2 and parent.co_name not in
                {"<listcomp>", "<dictcomp>", "<setcomp>", "<genexpr>"}):
            identity = "/".join(map(str, path)) + f":{parent.co_name}:{parent.co_firstlineno}"
            result.append((parent, identity))
        children = [value for value in parent.co_consts if isinstance(value, CodeType)]
        for index, child in enumerate(children):
            visit(child, (*path, index))

    visit(code)
    return result


def source_functions(path):
    """Inventory even bodies coalesced/dropped by the Python compiler.

    Python 3.10 can share one code object for byte-identical same-line lambdas.
    Such aliases cannot be distinguished by a profile event. Retain every AST
    definition in the denominator, credit at most one covered body in that
    group, and conservatively leave indistinguishable aliases uncovered.
    """
    path = Path(path)
    source = path.read_bytes()
    result = {identity for _, identity in code_functions(
        compile(source, str(path), "exec", dont_inherit=True))}
    compiled = Counter(tuple(identity.rsplit(":", 2)[1:]) for identity in result)
    declared = {}
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            name = "<lambda>" if isinstance(node, ast.Lambda) else node.name
            decorators = getattr(node, "decorator_list", [])
            line = min([node.lineno, *[value.lineno for value in decorators]])
            declared.setdefault((name, str(line)), []).append(node)
    for key, nodes in declared.items():
        for node in nodes[compiled[key]:]:
            result.add(f"unmapped:{node.col_offset}:{key[0]}:{key[1]}")
    return result


class FunctionCoverage:
    def __init__(self, roots, destination):
        self.roots = tuple(Path(root).resolve() for root in roots)
        self.destination = Path(destination)
        # Use object identity, not CodeType equality: Python 3.10 considers
        # byte-identical lambdas on the same line equal even if only one ran.
        self.identities = {}
        self.modules = {}
        self.filenames = {}
        self.hits = {}
        self.previous = None
        self.previous_thread = None

    def trace(self, frame, event, _argument):
        if event not in {"call", "return"}:
            return
        code = frame.f_code
        if code.co_filename.startswith("<"):
            return
        if code.co_filename not in self.filenames:
            path = Path(code.co_filename).resolve()
            self.filenames[code.co_filename] = str(path) if any(
                path.is_relative_to(root) for root in self.roots) else None
        filename = self.filenames[code.co_filename]
        if filename is None:
            return
        if event == "call" and code.co_name == "<module>" and id(code) not in self.modules:
            self.modules[id(code)] = code
            for function, identity in code_functions(code):
                self.identities[id(function)] = (function, filename, identity)
        found = self.identities.get(id(code))
        # Python 3.10 emits a spurious call/return pair when closing an
        # unstarted coroutine (f_lasti stays -1). A suspended generator's
        # return event, on the other hand, proves its body actually started.
        suspended = code.co_flags & (0x20 | 0x80 | 0x200)
        if found and ((event == "call" and not suspended)
                      or (event == "return" and frame.f_lasti >= 0)):
            self.hits.setdefault(found[1], set()).add(found[2])

    def start(self):
        self.previous = sys.getprofile()
        self.previous_thread = threading.getprofile()
        sys.setprofile(self.trace)
        threading.setprofile(self.trace)
        return self

    def stop(self):
        sys.setprofile(self.previous)
        threading.setprofile(self.previous_thread)

    def save(self):
        self.destination.parent.mkdir(parents=True, exist_ok=True)
        target = self.destination.with_name(f".functions.{os.getpid()}.{uuid.uuid4().hex}.json")
        target.write_text(json.dumps({"version": 1, "files": {
            name: sorted(hits) for name, hits in self.hits.items()}}) + "\n")
        return target


def process_startup():
    roots = os.environ.get("LITESIM_FUNCTION_COVERAGE_ROOTS")
    destination = os.environ.get("COVERAGE_FILE")
    if roots and destination:
        current = FunctionCoverage(json.loads(roots), destination).start()
        atexit.register(current.save)
