const systemFlow = [
  { step: 'MAP', title: '지도 작성', tech: 'Cartographer' },
  { step: 'NAVIGATE', title: '작업대 접근', tech: 'AMCL · Nav2 · NavFn · RPP' },
  { step: 'ALIGN', title: '로컬 정렬', tech: 'RGB-D 면 정렬 · 4WD' },
  { step: 'MANIPULATE', title: '양팔 조작', tech: 'DLS IK · SmolVLA' },
  { step: 'VERIFY', title: '결과 확인', tech: '엔코더 · 영상 · 실행 로그' },
];

const architectureLayers = [
  {
    code: 'N01',
    lane: 'NAVIGATION',
    title: 'Cartographer · AMCL · Nav2',
    language: 'C++ STACK + YAML',
    summary: '실차에서 지도를 만들고 저장 지도 기반 위치 추정과 작업대 접근을 수행했다.',
    detail: ['Cartographer mapping과 AMCL localization 분리', 'NavFn 전역 경로와 RPP 추종으로 실차 왕복', 'Collision Monitor와 4WD 구동 유지'],
    tone: 'blue',
  },
  {
    code: 'P02',
    lane: 'LOCAL PERCEPTION',
    title: 'RGB-D 면 정렬 · tf2 · 4WD',
    language: 'PYTHON',
    summary: 'RGB-D로 작업대 면을 찾고 차체 자세와 거리를 보정해 근접 접근한다.',
    detail: ['면 정렬과 정밀 주차를 실차에서 시험', '마지막 접근은 4WD 후진·회전과 연계', 'AprilTag 기반 작업셀 보정은 구현 완료로 표시하지 않음'],
    tone: 'cyan',
  },
  {
    code: 'M03',
    lane: 'MANIPULATION',
    title: 'DLS IK · FK · 엔코더 피드백',
    language: 'PYTHON',
    summary: '상대 DLS IK로 왼팔을 움직여 컵이 엔코더 피드백과 함께 상승하는 장면을 기록했다.',
    detail: ['현행 양팔 보정으로 계획 재생성 필요', '책상 완전 분리·운반·놓기는 검증 전', 'MoveIt 2 통합 완료로 확대하지 않음'],
    tone: 'blue',
  },
  {
    code: 'L04',
    lane: 'IMITATION LEARNING',
    title: 'SmolVLA · LeRobot · PyTorch',
    language: 'PYTHON',
    summary: '9000스텝 체크포인트를 로컬에 배포해 물 없는 실물 건식 rollout을 기록했다.',
    detail: ['체크포인트·관측 키·양팔 보정 확인', '실시간 추론과 실물 건식 동작 기록', '물 포함 반복 성공과 전체 미션 통합은 검증 전'],
    tone: 'cyan',
  },
  {
    code: 'S05',
    lane: 'EXECUTION SAFETY',
    title: '미션 상태기계 · 실행 backend',
    language: 'PYTHON + ROS 2',
    summary: '웹 주문을 ExecuteManipulationSkill Action과 Isaac 실행기로 연결한다.',
    detail: ['주문 큐·재시도·취소·timeout 상태 관리', 'ROS 2와 Isaac Python을 프로세스로 분리', '실차 이동과 양팔 조작의 전체 연결은 남은 과제'],
    tone: 'amber',
  },
  {
    code: 'H06',
    lane: 'HARDWARE BRIDGE',
    title: 'SO-101 ×2 · 4WD · 접촉센서',
    language: 'PYTHON + FIRMWARE',
    summary: '양팔과 4WD 실차는 움직였고, 힘·무게 계측은 아직 실행 경로에 연결되지 않았다.',
    detail: ['양팔 follower 보정과 엔코더 피드백 사용', '네 바퀴 엔코더와 같은 측 모터 동기 명령', '접촉센서는 5 V·GND만 연결, FSR·load cell 미연결'],
    tone: 'neutral',
  },
];

const interfaces = [
  ['NavigateToPose', 'Nav2', '작업대 staging pose 도착·취소·복구'],
  ['ExecuteManipulationSkill', 'Mission', 'phase·progress·failure code'],
  ['Unix socket JSON', 'Isaac executor', '명령·진행 feedback·물리 결과'],
  ['Order / Mission ID', 'Web control', '주문·backend·checkpoint 연결'],
  ['Execution evidence', 'Logger', 'timestamp·phase·code·artifact'],
];

const gates = [
  ['S0', 'SIMULATION', 'Isaac Sim 5.1 전체 왕복 12개 phase 물리 판정', '1회 통과'],
  ['N0-N4', 'NAVIGATION', 'Cartographer·AMCL·Nav2·RGB-D 면 정렬 실차 주행', '실차 기록'],
  ['G0', 'GRASP', '상대 DLS IK와 엔코더 피드백으로 컵 상승', '관찰 기록'],
  ['G1', 'MOTION', '컵 운반·놓기와 현행 보정 반복 검증', '남은 과제'],
  ['G2A', 'POUR', 'FSR·load cell 연결과 물 양 판정', '미연결'],
  ['D0 / A0', 'SMOLVLA', '9000스텝 정책의 실물 무수분 rollout', '건식 기록'],
  ['G3 / G4', 'INTEGRATION', '실차 이동·양팔 조작·계측의 전체 실물 미션', '미완료'],
];

const roles = [
  ['강사·멘토', '로컬 LLM·감독 에이전트', '교육·게이트 리뷰', 'MENTOR'],
  ['@mmporong', 'SLAM·Nav2·모바일 베이스', '로컬 정렬 인계·전체 통합', 'OWNER'],
  ['@Minsuk-ji', '세부 lane 확정 대기', 'MoveIt·ACT·하드웨어 협의', 'WRITE'],
  ['@jangjunseo05', '초대 수락 대기', 'MoveIt·ACT·하드웨어 협의', 'INVITED'],
];

export default function Home() {
  return (
    <main>
      <header className="site-header">
        <a className="brand" href="#top" aria-label="HOLD THE FLOW 처음으로">
          <span className="brand-mark" aria-hidden="true">HF</span>
          <span>HOLD THE FLOW</span>
        </a>
        <nav aria-label="주요 메뉴">
          <a href="#system">시스템</a>
          <a href="#architecture">구현</a>
          <a href="#roadmap">로드맵</a>
          <a href="#team">팀 운영</a>
        </nav>
        <a className="github-link" href="https://github.com/mmporong/bimanual-robot">
          GitHub 저장소
        </a>
      </header>

      <section className="hero" id="top">
        <div className="hero-copy">
          <p className="eyebrow">TEAM SOURCE OF TRUTH · CURRENT STATE</p>
          <h1>
            이동은 넓게,
            <br />
            조작은 정밀하게,
            <br />
            <span>결과는 계측한다.</span>
          </h1>
          <p className="hero-description">
            한 팔은 병의 흐름을 만들고, 다른 팔은 컵의 자세를 지킨다.
            ROS 2 위에서 실차 이동과 양팔 조작을 연결하고 있다. 주행·시뮬레이션·
            건식 조작의 확인 범위를 나눠 기록한다.
          </p>
          <div className="hero-actions">
            <a className="primary-action" href="#architecture">구현 구조 보기</a>
            <a className="secondary-action" href="#repository">팀 저장소 원칙</a>
          </div>
        </div>

        <aside className="project-brief" aria-label="프로젝트 핵심 정보">
          <div className="brief-heading">
            <span>PROJECT / 01</span>
            <strong>이동형 양팔 붓기</strong>
          </div>
          <dl>
            <div><dt>Robot</dt><dd>Mobile base + SO-101 ×2</dd></div>
            <div><dt>Runtime</dt><dd>ROS 2 Jazzy · Ubuntu 24.04</dd></div>
            <div><dt>Simulation</dt><dd>Isaac Sim 5.1</dd></div>
            <div><dt>Control</dt><dd>Nav2 · DLS IK · SmolVLA</dd></div>
          </dl>
          <p className="brief-note">전체 실물 물 서빙 미션은 아직 완료되지 않았다.</p>
        </aside>
      </section>

      <section className="system-rail" id="system" aria-labelledby="system-title">
        <div className="section-heading rail-heading">
          <div>
            <p className="eyebrow">END-TO-END SYSTEM RAIL</p>
            <h2 id="system-title">다섯 단계가 끊기지 않아야 성공이다.</h2>
          </div>
          <p>각 실행 단계의 결과와 실패 위치를 주문·미션 ID에 연결한다.</p>
        </div>

        <ol className="flow-list">
          {systemFlow.map((item, index) => (
            <li key={item.step}>
              <span className="flow-index">{String(index + 1).padStart(2, '0')}</span>
              <span className="flow-node" aria-hidden="true" />
              <span className="flow-step">{item.step}</span>
              <strong>{item.title}</strong>
              <small>{item.tech}</small>
            </li>
          ))}
        </ol>
      </section>

      <section className="team-source" id="repository">
        <p className="eyebrow">TEAM REPOSITORY</p>
        <h2>개인 기록이 아니라, 팀이 이어서 실행할 수 있는 기록.</h2>
        <p>
          설계·코드·실험 규격·회의 결정·검증 증거를 하나의 저장소에서
          관리한다. 현행 소스와 문서의 기준은 bimanual-robot 저장소의 main이며,
          실차·시뮬레이션·건식 시험의 근거와 한계를 함께 남긴다.
        </p>
      </section>

      <section className="architecture-section" id="architecture" aria-labelledby="architecture-title">
        <div className="section-heading split-heading">
          <div>
            <p className="eyebrow">IMPLEMENTATION ARCHITECTURE</p>
            <h2 id="architecture-title">기술을 나열하지 않고,<br />책임을 분리한다.</h2>
          </div>
          <p>
            구현된 기능과 계획 단계를 나눠 표시한다. 실차 주행, 시뮬레이션,
            양팔 조작은 서로 다른 검증 범위를 가진다.
          </p>
        </div>

        <div className="architecture-grid">
          {architectureLayers.map((layer) => (
            <article className={`architecture-card ${layer.tone}`} key={layer.code}>
              <div className="card-meta">
                <span>{layer.code}</span>
                <span>{layer.language}</span>
              </div>
              <p className="card-lane">{layer.lane}</p>
              <h3>{layer.title}</h3>
              <p className="card-summary">{layer.summary}</p>
              <ul>
                {layer.detail.map((item) => <li key={item}>{item}</li>)}
              </ul>
            </article>
          ))}
        </div>
      </section>

      <section className="language-section" aria-labelledby="language-title">
        <div className="section-heading split-heading">
          <div>
            <p className="eyebrow">LANGUAGE BOUNDARY</p>
            <h2 id="language-title">주행과 하드웨어를 묶고,<br />Python으로 실험을 잇는다.</h2>
          </div>
          <p>실차 주행 스택과 미션·인지·조작 도구의 현재 경계를 표시한다.</p>
        </div>
        <div className="language-map">
          <article>
            <span className="language-tag cpp">ROS 2 + FIRMWARE</span>
            <h3>실차 주행과 하드웨어 제어</h3>
            <p>Cartographer, Nav2, NavFn, RPP, Collision Monitor, 4WD base driver</p>
          </article>
          <div className="handoff-line" aria-hidden="true"><span>ROS 2 ACTION · TOPIC · TF</span></div>
          <article>
            <span className="language-tag python">PYTHON 3.12</span>
            <h3>미션·인지·조작 실험</h3>
            <p>mission manager, RGB-D 면 정렬, DLS IK, LeRobot SmolVLA, Isaac executor</p>
          </article>
        </div>
      </section>

      <section className="contract-section" aria-labelledby="contract-title">
        <div className="section-heading contract-heading">
          <div>
            <p className="eyebrow">ROS 2 CONTRACT</p>
            <h2 id="contract-title">완료와 실패가 보이는 인터페이스.</h2>
          </div>
          <p>장시간 실행과 취소가 필요한 동작은 Topic이 아니라 Action으로 연결한다.</p>
        </div>
        <div className="contract-table" role="table" aria-label="ROS 2 인터페이스">
          <div className="contract-row contract-head" role="row">
            <span role="columnheader">INTERFACE</span><span role="columnheader">OWNER</span><span role="columnheader">RESULT</span>
          </div>
          {interfaces.map(([name, owner, result]) => (
            <div className="contract-row" role="row" key={name}>
              <strong role="cell">{name}</strong><span role="cell">{owner}</span><span role="cell">{result}</span>
            </div>
          ))}
        </div>
      </section>

      <section className="simulation-section" aria-labelledby="simulation-title">
        <div className="simulation-number">5.1<span>ISAAC</span></div>
        <div>
          <p className="eyebrow">ISAAC SIM 5.1 · ONE PHYSICAL RUN</p>
          <h2 id="simulation-title">전체 왕복을 한 번 통과했다.<br />실물 성공과는 구분한다.</h2>
        </div>
        <div className="simulation-copy">
          <p>Isaac Sim 5.1의 한 world에서 도크 출발부터 조작·서빙·복귀까지 12개 phase와 물리 판정을 통과했다.</p>
          <ul>
            <li>구형 80 mm 강체 패드와 정답 좌표 조건</li>
            <li>Nav2와 충전 물리는 실행 범위 밖</li>
            <li>ROS 2 Action과 Isaac 실행기는 프로세스로 분리</li>
            <li>한 번의 시뮬레이션 통과를 반복 성공으로 확대하지 않음</li>
          </ul>
        </div>
      </section>

      <section className="roadmap-section" id="roadmap" aria-labelledby="roadmap-title">
        <div className="section-heading split-heading">
          <div>
            <p className="eyebrow">EVIDENCE-GATED ROADMAP</p>
            <h2 id="roadmap-title">한 번의 성공보다,<br />일곱 개의 통과 증거.</h2>
          </div>
          <p>완료·관찰·미연결·남은 과제를 같은 기준으로 구분한다.</p>
        </div>
        <div className="gate-list">
          {gates.map(([gate, lane, goal, status]) => (
            <article key={gate}>
              <span className="gate-code">{gate}</span>
              <span className="gate-lane">{lane}</span>
              <strong>{goal}</strong>
              <span className="gate-status">{status}</span>
            </article>
          ))}
        </div>
      </section>

      <section className="team-section" id="team" aria-labelledby="team-title">
        <div className="section-heading split-heading">
          <div>
            <p className="eyebrow">TEAM OWNERSHIP</p>
            <h2 id="team-title">권한과 담당을 분리해 기록한다.</h2>
          </div>
          <p>write 권한이 있다고 담당이 확정된 것은 아니다. lane은 회의에서 정하고 Issue에 남긴다.</p>
        </div>
        <div className="role-grid">
          {roles.map(([person, lead, shared, status]) => (
            <article key={person}>
              <div><strong>{person}</strong><span>{status}</span></div>
              <dl><dt>우선 담당</dt><dd>{lead}</dd><dt>공동 작업</dt><dd>{shared}</dd></dl>
            </article>
          ))}
        </div>
        <div className="workflow-rail" aria-label="팀 작업 순서">
          {['ISSUE', 'BRANCH', 'PULL REQUEST', 'LOCAL CHECK', 'MAIN'].map((item, index) => (
            <span key={item}><b>{String(index + 1).padStart(2, '0')}</b>{item}</span>
          ))}
        </div>
      </section>

      <section className="evidence-section" aria-labelledby="evidence-title">
        <div className="section-heading split-heading">
          <div>
            <p className="eyebrow">READ THE SOURCE</p>
            <h2 id="evidence-title">설계 근거와 구현 계약.</h2>
          </div>
          <p>사이트는 안내 화면이고, 최신 수정과 리뷰의 단일 원본은 GitHub 저장소다.</p>
        </div>
        <div className="document-grid">
          <a href="https://github.com/mmporong/bimanual-robot/blob/main/docs/architecture/20260903_IK_IL_%EA%B2%BD%EA%B3%84%EC%99%80_%EA%B2%B0%ED%95%A9.md">
            <span>DOC / ARCHITECTURE</span><strong>IK와 모방학습의 경계</strong><small>DLS IK 기준선과 정책 제어를 나누는 조작 설계</small>
          </a>
          <a href="https://github.com/mmporong/bimanual-robot/blob/main/docs/meetings/20260907_%EB%AC%BC%EC%84%9C%EB%B9%99%EB%A1%9C%EB%B4%87_%ED%9A%8C%EC%9D%98%EA%B2%B0%EC%A0%95%EA%B3%BC_%EC%8B%A4%ED%96%89%EB%B2%94%EC%9C%84.md">
            <span>DOC / MEETING</span><strong>회의 결정과 실행 범위</strong><small>물 서빙 MVP의 범위와 제외 항목</small>
          </a>
          <a href="https://github.com/mmporong/bimanual-robot/blob/main/docs/experiments/20261014_%EB%A1%9C%EC%BB%AC_SmolVLA_%EB%B0%B0%ED%8F%AC%EC%A0%84_%EA%B2%80%EC%A6%9D%EA%B3%BC_%EA%B1%B4%EC%8B%9D%EB%A1%A4%EC%95%84%EC%9B%83.md">
            <span>DOC / EXPERIMENT</span><strong>SmolVLA 건식 rollout</strong><small>9000스텝 체크포인트의 로컬 검증과 실물 기록</small>
          </a>
          <a href="https://github.com/mmporong/bimanual-robot/blob/main/docs/guides/JDAMR_RUNTIME.md">
            <span>DOC / GUIDE</span><strong>JD-AMR 실차 실행법</strong><small>Cartographer·Nav2·RGB-D 정렬과 주행 통합</small>
          </a>
        </div>
      </section>

      <footer>
        <div><span className="brand-mark" aria-hidden="true">HF</span><strong>HOLD THE FLOW</strong></div>
        <p>Mobile Bimanual Pouring · Team Repository · 2026</p>
        <a href="https://github.com/mmporong/bimanual-robot">mmporong/bimanual-robot</a>
      </footer>
    </main>
  );
}
