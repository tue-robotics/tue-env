#! /usr/bin/env python3

from __future__ import print_function

import os
import sys
import xml.etree.ElementTree as ET

import regex
import operator


_condition_expression = None
_operator_expression = None
_split_space_expression = None
_split_or_expression = None
_split_and_expression = None


def _get_condition_expression():
    # type: () -> regex.Regex
    global _condition_expression
    if not _condition_expression:
        _condition_expression = regex.compile(r'(?<item>((?<var>\$\w+) *(?<op>==|!=|>=|>|<=|<) *(?<value>[\w-]+))|((?&value) *(?&op) *(?&var)))( *(and|or) *((?R)|(?&item)))*')
    return _condition_expression


def _get_operator_expression():
    # type: () -> regex.Regex
    global _operator_expression
    if not _operator_expression:
        _operator_expression = regex.compile(r' *(?<op>==|!=|>=|>|<=|<) *')
    return _operator_expression


def _get_split_space_expression():
    # type: () -> regex.Regex
    global _split_space_expression
    if not _split_space_expression:
        _split_space_expression = regex.compile(r' *')
    return _split_space_expression


def _get_split_or_expression():
    # type: () -> regex.Regex
    global _split_or_expression
    if not _split_or_expression:
        _split_or_expression = regex.compile(r' *or *')
    return _split_or_expression


def _get_split_and_expression():
    # type: () -> regex.Regex
    global _split_and_expression
    if not _split_and_expression:
        _split_and_expression = regex.compile(r' *and *')
    return _split_and_expression


def dep_evaluate_condition(dep):
    # type: (ET.Element) -> bool
    """
    Wrapper to call evaluate_condition with os.environment on a ET.Element

    :param dep: ET.Element with an optional 'condition' attribute
    :return: Condition is satisfied or not
    """
    return evaluate_condition(dep.attrib.get('condition', None), os.environ)


def evaluate_condition(condition, context):
    # type: (Union[str, None], dict) -> bool
    """
    Evaluate condition against provided context

    :param condition: condition to evaluate
    :param context: dictionary on which to evaluate, probably os.environ
    :return: Condition is satisfied or not
    """
    if condition is None:
        return True

    expr = _get_condition_expression()
    try:
        parse_results = expr.fullmatch(condition, concurrent=True)
    except Exception as e:
        raise ValueError("condition '{}' failed to parse: {}".format(condition, e))

    if parse_results is None:
        raise ValueError("condition '{}' failed to parse".format(condition))

    # Make sure there is at least one space surrounding the operator, so we can split on spaces later on
    sub = _get_operator_expression()
    try:
        parse_results = sub.sub(r' \g<op> ', parse_results.string, concurrent=True)
    except Exception as e:
        raise ValueError("'{}' failed to substitute whitespace: {}".format(parse_results.string, e))

    # First split on 'or', so only first part needs to be evaluated in case it evaluates to True
    parsed_list = split_on_or(parse_results)
    return _evaluate(parsed_list, context)


def split_on_or(s):
    # type: (str) -> list[Union[str, list]]
    """
    Split string on 'or' surround by any number of spaces, other whitespaces are ignored

    :param s: string to split
    :return: list of strings
    """
    split_or = _get_split_or_expression()
    l = split_or.split(s)
    for idx, v in enumerate(l):
        if " and " in v or " or " in v:
            l[idx] = split_on_and(v)
        else:
            l[idx] = split_on_space(v)
    return flat_to_nested(l, 'or')


def split_on_and(s):
    # type: (str) -> list[Union[str, list]]
    """
    Split string on 'and' surround by any number of spaces, other whitespaces are ignored

    :param s: string to split
    :return: list of strings
    """
    split_and = _get_split_and_expression()
    l = split_and.split(s)
    for idx, v in enumerate(l):
        if " and " in v or " or " in v:
            l[idx] = split_on_and(v)
        else:
            l[idx] = split_on_space(v)
    return flat_to_nested(l, 'and')


def split_on_space(s):
    # type: (str) -> list[str]
    """
    Split string on any length of spaces, not splitting on other whitespaces

    :param s: string to split
    :return: list of strings
    """
    split_space = _get_split_space_expression()
    return split_space.split(s, concurrent=True)


def flat_to_nested(fl, add):
    """
    [a, b, c, d] --> [[[a, add, b], add, c,], add, d]

    :param fl: flat list of strings/lists
    :param add: string item be added in between list items
    :return: nested list
    """
    if len(fl) == 1:
        return fl[0]
    return [flat_to_nested(fl[:-1], add), add, fl[-1]]


def _evaluate(parse_results, context):
    # type: (Union[str, List[str]]) -> Union[bool, str]
    if not isinstance(parse_results, list):
        if parse_results.startswith('$'):
            # get variable from context
            return str(context.get(parse_results[1:], ''))
        # return literal value
        return parse_results

    # recursion
    assert len(parse_results) == 3

    # handle logical operators
    if parse_results[1] == 'and':
        return _evaluate(parse_results[0], context) and \
            _evaluate(parse_results[2], context)
    if parse_results[1] == 'or':
        return _evaluate(parse_results[0], context) or \
            _evaluate(parse_results[2], context)

    # handle comparison operators
    operators = {
        '==': operator.eq,
        '!=': operator.ne,
        '<=': operator.le,
        '<': operator.lt,
        '>=': operator.ge,
        '>': operator.gt,
    }
    assert parse_results[1] in operators.keys()

    return operators[parse_results[1]](
        _evaluate(parse_results[0], context),
        _evaluate(parse_results[2], context))


def main():
    if len(sys.argv) != 2:
        print("Usage: parse-package-xml PACKAGE.XML")
        return 1

    tree = ET.parse(sys.argv[1])
    doc = tree.getroot()

    dep_set = set()

    deps = doc.findall('build_depend')
    dep_set |= {dep.text for dep in deps if dep_evaluate_condition(dep)}

    deps = doc.findall('buildtool_depend')
    dep_set |= {dep.text for dep in deps if dep_evaluate_condition(dep)}

    deps = doc.findall('build_export_depend')
    dep_set |= {dep.text for dep in deps if dep_evaluate_condition(dep)}

    deps = doc.findall('buildtool_export_depend')
    dep_set |= {dep.text for dep in deps if dep_evaluate_condition(dep)}

    deps = doc.findall('exec_depend')
    dep_set |= {dep.text for dep in deps if dep_evaluate_condition(dep)}

    deps = doc.findall('depend')
    dep_set |= {dep.text for dep in deps if dep_evaluate_condition(dep)}

    deps = doc.findall('run_depend')
    dep_set |= {dep.text for dep in deps if dep_evaluate_condition(dep)}

    if os.getenv('TUE_INSTALL_TEST_DEPEND', 'false') == 'true':
        deps = doc.findall('test_depend')
        dep_set |= {dep.text for dep in deps if dep_evaluate_condition(dep)}

    if os.getenv('TUE_INSTALL_DOC_DEPEND', 'false') == 'true':
        deps = doc.findall('doc_depend')
        dep_set |= {dep.text for dep in deps if dep_evaluate_condition(dep)}

    print('\n'.join(dep_set))


if __name__ == "__main__":
    sys.exit(main())
