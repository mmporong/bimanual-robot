const systemFlow = [
  { step: 'MAP', title: '지도 작성', tech: 'SLAM Toolbox' },
  { step: 'NAVIGATE', title: '작업대 접근', tech: 'AMCL · Nav2' },
  { step: 'ALIGN', title: '로컬 정렬', tech: 'RGB-D · AprilTag' },
  { step: 'MANIPULATE', title: '양팔 조작', tech: 'MoveIt 2 · ACT' },
  { step: 'VERIFY', title: '결과 계측', tech: 'FSR · Load cell' },
];

const architectureLayers = [
  {
    code: 'N01',
    lane: 'NAVIGATION',
    title: 'SLAM Toolbox · AMCL · Nav2',
    language: 'C++ STACK + YAML',
    summary: '지도를 만들고, 저장 지도에서 위치를 추정한 뒤 작업대 staging pose까지 이동한다.',
    detail: ['mapping과 localization을 동시에 실행하지 않음', 'NavigateToPose 결과·도달 오차·복구 횟수 기록', '조작 전 zero velocity 확인'],
    tone: 'blue',
  },
  {
    code: 'P02',
    lane: 'LOCAL PERCEPTION',
    title: 'RGB-D · AprilTag · tf2',
    language: 'PYTHON',
    summary: 'Nav2가 넘긴 거친 위치를 양팔 작업공간에 맞는 workcell 좌표로 보정한다.',
    detail: ['base_link→camera→workcell TF 검사', '검출 시각의 timestamp로 좌표 조회', '투명 컵은 표식·도킹 지그로 보완'],
    tone: 'cyan',
  },
  {
    code: 'M03',
    lane: 'MANIPULATION',
    title: 'MoveIt 2 · IK · Planning Scene',
    language: 'C++',
    summary: '파지·기울임·복귀의 결정론적 기준선을 만들고 양팔 충돌을 검사한다.',
    detail: ['left_arm·right_arm·both_arms planning group', '테이블·병·컵·도킹 패드 충돌 객체', '하나의 양팔 FollowJointTrajectory로 동기 실행'],
    tone: 'blue',
  },
  {
    code: 'L04',
    lane: 'IMITATION LEARNING',
    title: 'LeRobot ACT · PyTorch',
    language: 'PYTHON',
    summary: '실물 양팔 시연에서 조작 구간만 학습하고 같은 조건의 IK 기준선과 비교한다.',
    detail: ['RGB 3시점 + 좌우 관절 상태', '5 episode smoke → 20 episode 과적합', '실물 데이터와 합성 데이터 분리'],
    tone: 'cyan',
  },
  {
    code: 'S05',
    lane: 'EXECUTION SAFETY',
    title: 'command_mux · safety_guard',
    language: 'C++',
    summary: 'MoveIt·ACT·텔레옵 중 한 명령 소유자만 허용하고 실행 직전 위험을 차단한다.',
    detail: ['관절 범위·변화량·타임아웃 검사', '팔 간 최소 거리와 FSR 과압 확인', '위반 시 HOLD 또는 SAFE_RETURN'],
    tone: 'amber',
  },
  {
    code: 'H06',
    lane: 'HARDWARE BRIDGE',
    title: 'SO-101 · FSR · Load cell',
    language: 'PYTHON → C++',
    summary: '하나의 bridge가 좌우 시리얼 포트를 소유하고 센서 시간축을 ROS 2에 연결한다.',
    detail: ['LeRobot bi_so_follower로 양팔 포트 단독 소유', '원시값·필터값·sequence 동시 기록', '검증 뒤 ros2_control SystemInterface 확장'],
    tone: 'neutral',
  },
];

const interfaces = [
  ['NavigateToPose', 'Nav2', '작업대 staging pose 도착·취소·복구'],
  ['AlignToWorkcell', 'Perception', 'workcell pose·오차·confidence'],
  ['ExecuteBimanualSkill', 'Motion', 'stage·progress·failure code'],
  ['FollowJointTrajectory', 'Trajectory server', '좌우 오차·tolerance·동기 취소'],
  ['EpisodeEvent', 'Logger', 'timestamp·stage·code·evidence'],
];

const gates = [
  ['S0', 'SIM ENV', 'Isaac Sim 6.0 실행 장비와 ROS 2 Bridge 확인', '장비 대기'],
  ['N0–N4', 'NAVIGATION', 'TF·지도·AMCL·작업대 반복 접근', '실측 전'],
  ['G0', 'GRASP', '병·컵 정적 파지와 FSR 판정', '실측 전'],
  ['G1', 'MOTION', 'MoveIt 양팔 무수 기울임과 안전 복귀', '실측 전'],
  ['G2A', 'POUR', '건식 붓기와 병·컵 도킹 저울 판정', '실측 전'],
  ['D0 / A0', 'ACT', '5회 수집 smoke와 20회 과적합 기준선', '실측 전'],
  ['G3 / G4', 'INTEGRATION', '이동→조작 통합과 실패 복구', '실측 전'],
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
          <p className="eyebrow">TEAM SOURCE OF TRUTH · PLAN A</p>
          <h1>
            이동은 넓게,
            <br />
            조작은 정밀하게,
            <br />
            <span>결과는 계측한다.</span>
          </h1>
          <p className="hero-description">
            한 팔은 병의 흐름을 만들고, 다른 팔은 컵의 자세를 지킨다.
            ROS 2 위에서 이동·인지·조작·학습·검증을 하나의 재현 가능한
            시스템으로 연결한다.
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
            <div><dt>Simulation</dt><dd>Isaac Sim 6.0</dd></div>
            <div><dt>Control</dt><dd>Nav2 · MoveIt 2 · ACT</dd></div>
          </dl>
          <p className="brief-note">실물 검증 전 수치는 계획으로 표시한다.</p>
        </aside>
      </section>

      <section className="system-rail" id="system" aria-labelledby="system-title">
        <div className="section-heading rail-heading">
          <div>
            <p className="eyebrow">END-TO-END SYSTEM RAIL</p>
            <h2 id="system-title">다섯 단계가 끊기지 않아야 성공이다.</h2>
          </div>
          <p>각 단계의 Action 결과와 실패 위치를 한 episode로 기록한다.</p>
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
          관리한다. 모든 변경은 Issue와 작업 브랜치, Pull Request, 로컬
          검증 기록을 거쳐 main에 반영한다. 팀원 승인은 필수가 아니다.
        </p>
      </section>

      <section className="architecture-section" id="architecture" aria-labelledby="architecture-title">
        <div className="section-heading split-heading">
          <div>
            <p className="eyebrow">IMPLEMENTATION ARCHITECTURE</p>
            <h2 id="architecture-title">기술을 나열하지 않고,<br />책임을 분리한다.</h2>
          </div>
          <p>
            검증된 ROS 2 스택은 설정하고, 프로젝트 고유의 조정·안전·학습·계측
            계층은 직접 구현한다.
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
            <h2 id="language-title">C++은 실행을 지키고,<br />Python은 실험을 바꾼다.</h2>
          </div>
          <p>언어 선택은 취향이 아니라 변경 주기와 안전 책임에 따라 나눈다.</p>
        </div>
        <div className="language-map">
          <article>
            <span className="language-tag cpp">C++17</span>
            <h3>시간 제약과 공통 안전 경로</h3>
            <p>command mux, safety guard, 양팔 trajectory server, IK·충돌 검사, 센서 필터</p>
          </article>
          <div className="handoff-line" aria-hidden="true"><span>ROS 2 ACTION · TOPIC · TF</span></div>
          <article>
            <span className="language-tag python">PYTHON 3.12</span>
            <h3>인지·학습·실험 자동화</h3>
            <p>mission manager, RGB-D, LeRobot bridge, ACT 학습·추론, episode 평가</p>
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
        <div className="simulation-number">16<span>GB</span></div>
        <div>
          <p className="eyebrow">ISAAC SIM 6.0 · S0 GATE</p>
          <h2 id="simulation-title">시뮬레이터는 결정했다.<br />실행 장비는 검증해야 한다.</h2>
        </div>
        <div className="simulation-copy">
          <p>현재 개발 노트북의 VRAM은 8GB로 공식 최소 16GB에 미달한다. ROS 2 패키지와 URDF는 이 장비에서 준비하고, Compatibility Checker를 통과한 워크스테이션에서 실행한다.</p>
          <ul>
            <li>URDF → USD 관절 축·관성·collider 확인</li>
            <li>실물과 동일한 Topic·Action·TF 사용</li>
            <li>sim-only·real-only·mixed 데이터 분리 평가</li>
            <li>시뮬레이션 성공을 실물 성공으로 표기하지 않음</li>
          </ul>
        </div>
      </section>

      <section className="roadmap-section" id="roadmap" aria-labelledby="roadmap-title">
        <div className="section-heading split-heading">
          <div>
            <p className="eyebrow">EVIDENCE-GATED ROADMAP</p>
            <h2 id="roadmap-title">한 번의 성공보다,<br />일곱 개의 통과 증거.</h2>
          </div>
          <p>아래 상태는 계획이다. 실물 로그와 영상이 남은 뒤에만 완료로 바꾼다.</p>
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
          <a href="https://github.com/mmporong/bimanual-robot/blob/docs/plan-a-pouring-research/docs/20260901_%EA%B5%AC%ED%98%84%EC%95%84%ED%82%A4%ED%85%8D%EC%B2%98_ROS2_CPP_Python_ACT_IsaacSim.md">
            <span>DOC / ARCHITECTURE</span><strong>ROS 2·C++·Python 구현 아키텍처</strong><small>Nav2, MoveIt 2, ACT, Isaac Sim의 실제 연결 계약</small>
          </a>
          <a href="https://github.com/mmporong/bimanual-robot/blob/docs/plan-a-pouring-research/docs/20260828_1%EC%95%88_%ED%99%95%EC%A0%95_%EC%9D%B4%EB%8F%99%ED%98%95_%EC%96%91%ED%8C%94_%EB%B6%93%EA%B8%B0.md">
            <span>DOC / PLAN A</span><strong>HOLD THE FLOW 1안 확정</strong><small>시나리오, 하드웨어 전제, 게이트와 역할</small>
          </a>
          <a href="https://github.com/mmporong/bimanual-robot/blob/docs/plan-a-pouring-research/research/R31_1%EC%95%88_%EC%9D%B4%EB%8F%99%ED%98%95_%EC%96%91%ED%8C%94_%EB%B6%93%EA%B8%B0_%EB%85%BC%EB%AC%B8%EC%A0%81%EC%9A%A9.md">
            <span>RESEARCH / R31</span><strong>논문 적용 설계</strong><small>센서 책임, 평가 지표, 실패 데이터와 연구 근거</small>
          </a>
          <a href="https://github.com/mmporong/bimanual-robot/blob/docs/plan-a-pouring-research/docs/TEAM_WORKFLOW.md">
            <span>DOC / TEAM</span><strong>협업과 작업 관리</strong><small>초대, Issue, 브랜치, 리뷰와 완료 증거</small>
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
