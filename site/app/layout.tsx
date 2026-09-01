import type { Metadata } from 'next';
import './globals.css';

export const metadata: Metadata = {
  title: 'HOLD THE FLOW · 이동형 양팔 로봇',
  description:
    'ROS 2, Nav2, MoveIt 2, ACT, Isaac Sim으로 구현하는 이동형 양팔 붓기 팀 프로젝트',
  openGraph: {
    title: 'HOLD THE FLOW · 이동형 양팔 로봇',
    description:
      '이동·인지·양팔 조작·계측 검증을 하나로 연결하는 팀 프로젝트',
    type: 'website',
  },
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="ko">
      <body>{children}</body>
    </html>
  );
}
