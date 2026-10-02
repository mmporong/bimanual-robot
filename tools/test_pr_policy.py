import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('pr_policy', ROOT/'tools/ci/pr_policy.py')
assert spec is not None and spec.loader is not None
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


@pytest.mark.parametrize('paths,risk', [
    (['README.md', 'docs/a.md'], 'low'), (['tools/a.py'], 'medium'),
    (['src/a.py'], 'high'), (['design/mechanical/a.yaml'], 'high'),
    (['.github/workflows/a.yml'], 'automation'),
    (['firmware/a.cpp', '.github/a'], 'hardware'),
])
def test_classify(paths, risk):
    assert policy.classify(paths)[0] == risk


@pytest.mark.parametrize('risk_path,draft,existing_auto,expected', [
    ('README.md', False, None, '--auto'),
    ('README.md', True, None, None),
    ('README.md', True, {}, '--disable-auto'),
    ('src/a.py', False, {}, None),
    ('src/a.py', False, None, '--auto'),
    ('.github/a.yml', False, None, '--auto'),
])
def test_policy_reports_without_comment_or_mail_api(tmp_path, monkeypatch,
                                                   risk_path, draft, existing_auto, expected):
    calls = []
    def fake_gh(*args):
        calls.append(args)
        if args[:2] == ('pr', 'view'):
            return json.dumps(dict(isDraft=draft, autoMergeRequest=existing_auto,
                                   headRefOid='abc123', labels=[{'name': 'risk:low'}]))
        if args[0] == 'api':
            assert args[-1].endswith('/files?per_page=100')
            return json.dumps([[dict(filename=risk_path)]])
        return ''
    monkeypatch.setattr(policy, 'gh', fake_gh)
    policy.run(51, 'owner/repo', tmp_path/'summary.md')
    merges = [args for args in calls if args[:2] == ('pr', 'merge')]
    assert (len(merges) == 1 and expected in merges[0]) if expected else not merges
    assert (tmp_path/'summary.md').stat().st_size > 0
    assert all(args[:2] != ('pr', 'comment') for args in calls)
    assert all(args[:2] != ('api', '--method') for args in calls)


def test_all_file_pages_classified(tmp_path, monkeypatch):
    def fake_gh(*args):
        if args[:2] == ('pr', 'view'):
            return json.dumps(dict(isDraft=False, autoMergeRequest=None, headRefOid='abc', labels=[]))
        if args[0] == 'api':
            return json.dumps([[dict(filename='docs/a.md')], [dict(filename='firmware/b.cpp')]])
        return ''
    monkeypatch.setattr(policy, 'gh', fake_gh)
    policy.run(1, 'owner/repo', tmp_path/'summary.md')


def test_empty_file_list_fails_closed(tmp_path, monkeypatch):
    monkeypatch.setattr(policy, 'gh', lambda *args: '{}' if args[0] == 'pr' else '[]')
    with pytest.raises(ValueError):
        policy.run(1, 'owner/repo', tmp_path/'summary.md')


def test_workflows_do_not_publish_conversation_comments():
    for name in ('auto-pr-assist.yml', 'pr-assist.yml'):
        workflow = yaml.safe_load((ROOT/'.github/workflows'/name).read_text())
        scripts = '\n'.join(step.get('run', '') for job in workflow['jobs'].values() for step in job['steps'])
        assert '/comments' not in scripts
        assert 'gh pr comment' not in scripts
        assert 'tools/ci/pr_policy.py' in scripts


def test_target_workflow_runs_only_base_policy():
    workflow = yaml.safe_load((ROOT/'.github/workflows/pr-assist.yml').read_text())
    checkout = workflow['jobs']['classify']['steps'][0]
    assert checkout['with']['ref'] == '${{ github.event.pull_request.base.sha }}'
    assert checkout['with']['persist-credentials'] is False


def test_failure_summary_uses_actual_failed_steps(tmp_path):
    workflow = yaml.safe_load((ROOT/'.github/workflows/pr-gate.yml').read_text())
    step = workflow['jobs']['validate']['steps'][-1]
    assert step['if'] == 'failure()'
    output = tmp_path/'summary.md'
    subprocess.run(['bash', '-eu', '-c', step['run']], check=True, env={
        **os.environ, 'GITHUB_STEP_SUMMARY': str(output),
        'STEP_RESULTS': json.dumps({'tests': {'outcome': 'failure'}, 'model': {'outcome': 'skipped'}})})
    report = output.read_text()
    assert 'python3 -m pytest -q <파일::테스트>' in report
    assert 'validate_description.py' not in report


def test_workflow_shell_syntax():
    for path in (ROOT/'.github/workflows').glob('*.yml'):
        workflow = yaml.safe_load(path.read_text())
        for job in workflow['jobs'].values():
            for step in job['steps']:
                if 'run' in step:
                    subprocess.run(['bash', '-n'], input=step['run'], text=True, check=True)
