"""Classify PRs and write actionable Actions summaries without comments."""
from __future__ import annotations

import html
import json
import os
from pathlib import Path
import subprocess


RISKS = ('low', 'medium', 'high', 'hardware', 'automation')


def classify(paths):
    for risk, prefixes in (
        ('hardware', ('firmware/', 'tools/servo/')),
        ('automation', ('.github/',)),
        ('high', ('src/', 'design/mechanical/')),
    ):
        matched = [p for p in paths if p.startswith(prefixes)]
        if matched:
            return risk, matched
    matched = [p for p in paths if not (p.startswith('docs/') or p == 'README.md')]
    return ('medium', matched) if matched else ('low', paths)


def summary(risk, paths, number, draft):
    checks = {
        'low': '문서 링크와 README 동기화를 확인한다.',
        'medium': '변경 기능의 회귀 테스트를 보완하고 전체 테스트를 실행한다.',
        'high': '좌표·단위·관절 한계·충돌·관성 변경을 확인하고 관련 시뮬레이션 근거를 검토한다.',
        'hardware': '장치 주소·단위·통신 오류·정지 처리와 설정 변경을 검토하고 mock 회귀 테스트를 실행한다.',
        'automation': '이벤트·권한·외부 전송·fork 신뢰 경계를 검토하고 정책 회귀 테스트를 실행한다.',
    }
    disposition = ('Draft 해제 후 필수 검사 통과 시 자동 병합 예약 가능.' if draft else
                   '필수 검사 통과 후 자동 병합 예약. 경로 분류는 병합 차단 조건이 아니다.')
    lines = [f'## PR #{number} 검토 안내', '', f'분류: `risk:{risk}`. {disposition}', '',
             '### 분류 근거', '', *['- <code>'+html.escape(p)+'</code>' for p in paths], '',
             '### 해결·검증 순서', '',
             f'1. {checks[risk]}',
             f'2. `gh pr checks {number}`로 실패한 검사를 확인한다. 실패가 있으면 해당 run의 '
             '`gh run view <run-id> --log-failed`로 원인을 읽고 수정한다.',
             '3. 저장소 루트에서 다음 검사를 통과시킨 뒤 변경 파일만 커밋·push한다.', '',
             '```bash', 'python3 -m pytest -q',
             'python3 tools/ci/check_local_markdown_links.py',
             'python3 tools/ci/sync_readme_status.py',
             'python3 src/hold_flow_description/scripts/validate_description.py', '```', '']
    lines += ['4. 충돌이 있으면 main 변경을 작업 브랜치에 반영하고 같은 검사를 다시 실행한다. '
              '검사 통과 뒤 GitHub가 자동 병합한다. 보호 규칙은 유지한다.', '']
    return '\n'.join(lines)


def gh(*args):
    return subprocess.check_output(['gh', *args], text=True)


def run(number, repo, summary_path):
    metadata = json.loads(gh('pr', 'view', str(number), '--repo', repo, '--json',
                             'isDraft,autoMergeRequest,headRefOid,labels'))
    pages = json.loads(gh('api', '--paginate', '--slurp',
                         f'repos/{repo}/pulls/{number}/files?per_page=100'))
    paths = [f['filename'] for page in pages for f in page]
    if not paths:
        raise ValueError('PR file list is empty; refusing automatic merge')
    risk, matched = classify(paths)
    # No notification/comment API: the only report sink is the Actions summary.
    with Path(summary_path).open('a', encoding='utf-8') as stream:
        stream.write(summary(risk, matched, number, metadata['isDraft']))
    existing = {label['name'] for label in metadata['labels']}
    desired = 'risk:' + risk
    removals = sorted(existing & {'risk:' + r for r in RISKS} - {desired})
    if removals or desired not in existing:
        args = ['pr', 'edit', str(number), '--repo', repo]
        for label in removals:
            args += ['--remove-label', label]
        if desired not in existing:
            args += ['--add-label', desired]
        gh(*args)
    if metadata['isDraft'] and metadata['autoMergeRequest'] is not None:
        gh('pr', 'merge', str(number), '--repo', repo, '--disable-auto')
    elif not metadata['isDraft'] and metadata['autoMergeRequest'] is None:
        gh('pr', 'merge', str(number), '--repo', repo, '--auto', '--squash',
           '--match-head-commit', metadata['headRefOid'])


if __name__ == '__main__':
    run(int(os.environ['PR_NUMBER']), os.environ['GH_REPO'], os.environ['GITHUB_STEP_SUMMARY'])
