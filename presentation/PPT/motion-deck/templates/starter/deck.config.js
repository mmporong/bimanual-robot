// Deck settings read by the motion-deck engine. Paths are relative to this folder unless absolute or ~/.
module.exports = {
  title: '프로젝트 최종 발표',
  author: 'MS Team',
  company: 'MS Team',
  theme: 'sky', // sky | teal | blue; fine-tune single colours with palette: { main: '1690D6', ... }
  // assetsDir: '~/Downloads/발표자료/<날짜>_<이름>/_build/assets',   // default: ./assets
  brand: {
    mark: 'logo_mark_light_sky.png', markW: 0.44, markH: 0.418, alt: 'MS Team 로고', // header, every content slide
    wordmark: 'logo_h_light_sky.png', wordW: 2.68, wordH: 0.55, // title + closing slides
  },
};
