import {failureTasks} from './failures.js';
const $=id=>document.getElementById(id);
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
export function duration(seconds){if(seconds===null||seconds===undefined||!Number.isFinite(Number(seconds)))return '未记录';let n=Math.round(Number(seconds));return `${Math.floor(n/60)}分${String(n%60).padStart(2,'0')}秒`}
function kpi(label,value,unit,note){return `<div class="kpi"><div class="label">${esc(label)}</div><div class="value">${esc(value)}<span>${esc(unit)}</span></div><small>${esc(note)}</small></div>`}
export function cardMarkup(c){const m=c.metrics??{},stages=m.stages??[],total=stages.reduce((a,r)=>a+(r.active_seconds??0),0);return `<article class="card"><a class="preview" href="?case=${encodeURIComponent(c.id)}">${c.thumbnail?`<img loading="lazy" src="${esc(c.thumbnail)}" alt="${esc(c.title)}的原始机械模型">`:`<div class="placeholder">${esc(c.title)}</div>`}<span class="preview-label">${esc(c.revision)} / ORIGINAL GEOMETRY</span><span class="preview-play">▶</span></a><div class="card-body"><div class="card-top"><h2>${esc(c.title)}</h2><span class="status ${m.development_passed?'good':''}">${esc(m.card_badge??c.status_label)}</span></div><p>${esc(c.summary)}</p>${c.failure_display?.card_summary?`<p class="card-failure ${c.failure_display.card_kind??'timeout'}"><b>哪里没过</b>${esc(c.failure_display.card_summary)}</p>`:""}<div class="card-facts"><span><b>${esc(m.physical_parts??'—')}</b> 实物零件</span><span><b>${esc(m.body_count??'—')}</b> 仿真刚体</span><span><b>${esc(m.formal_passed??0)}/${esc(m.task_count??'—')}</b> 正式任务</span></div><div class="timeline-bar">${stages.map((r,i)=>`<i class="${['original','first','second'][i]}" style="width:${total?100*(r.active_seconds??0)/total:0}%" title="${esc(r.title)}：${duration(r.active_seconds)}"></i>`).join('')}</div><div class="meta"><span>累计 active ${duration(total)}</span><a class="open" href="?case=${encodeURIComponent(c.id)}">打开模型与回放 ↗</a></div></div></article>`}

export async function renderBriefing(){
  let briefing;
  try{briefing=await (await fetch('briefing.json',{cache:'no-store'})).json()}
  catch(e){return}
  const statusLabel={open:'仍有缺口',code_fixed_score_frozen:'代码已修 · 成绩未追改',closed:'已关闭'};
  const bucketLabel={mechanical:'机械',process:'流程',mixed:'试算过 / 正式未收'};
  $('briefing-headline').textContent=briefing.headline;
  $('briefing-policy').textContent=briefing.read_policy||'';
  const cr=document.getElementById('compare-rule'); if(cr) cr.textContent=briefing.compare_rule||'';
  $('machine-strip').innerHTML=briefing.machines.map(m=>{const f=m.four||{};return `<div class="machine-chip ${esc(m.bucket)}"><a href="?case=${encodeURIComponent(m.id)}"><span class="rev">${esc(m.rev)} · 主列 ${esc(f.primary||'')}</span><b>${esc(m.title)}</b><em>正式 ${esc(f.formal||m.formal)}</em><small>${esc(bucketLabel[m.bucket]||m.bucket)}</small><p>${esc(m.one_liner)}</p></a><div class="four-mini"><span>功能 ${esc(f.functional||'—')}</span><span>未证 ${esc(f.unproven||'—')}</span><span>过严 ${esc(f.too_strict||'—')}</span></div>${m.repro?`<a class="repro-link" href="${esc(m.repro)}" target="_blank">复现包 ↗</a>`:""}</div>`}).join('');
  for(const cls of briefing.classes||[]){
    const host=document.getElementById('class-'+cls.id); if(!host) continue;
    host.innerHTML=`<p class="eyebrow">${cls.id==='mechanical'?'MECHANICAL':'PROCESS / BUDGET'}</p><h2>${esc(cls.title)}</h2><p class="class-note">${esc(cls.note)}</p>`+(cls.items||[]).map(it=>`<a class="class-item ${esc(it.kind)}" href="${esc(it.href)}"><b>${esc(it.title)}</b><p>${esc(it.line)}</p></a>`).join('');
  }
  $('contract-grid').innerHTML=(briefing.contracts||[]).map(c=>`<div class="contract ${esc(c.status)}"><span class="n">${esc(c.id)}</span><div><b>${esc(c.title)}</b><small>${esc(statusLabel[c.status]||c.status)}</small><p>${esc(c.line)}</p></div></div>`).join('');

  const q=document.getElementById('open-questions');
  if(q){q.innerHTML=(briefing.open_questions||[]).map(s=>`<li>${esc(s)}</li>`).join('');}
  const idx=document.getElementById('repro-index');
  if(idx && briefing.repro_index){idx.href=briefing.repro_index; idx.hidden=false}

  const paper=briefing.paper;
  if(paper){
    const g=document.getElementById('paper-gap'); if(g) g.textContent=paper.gap||'';
    const lz=document.getElementById('lz-cares');
    if(lz) lz.innerHTML=(paper.lz_cares||[]).map((s,i)=>`<li><b>刘圳 ${i+1}.</b> ${esc(s)}</li>`).join('')+(paper.lz_extras||[]).map(s=>`<li>${esc(s)}</li>`).join('');
    const et=document.getElementById('e-table');
    if(et) et.innerHTML=(paper.e_table||[]).map(r=>`<div class="e-row"><b>${esc(r.id)}</b><span>${esc(r.maps)}</span><em>${esc(r.status)}</em><p>${esc(r.paper)} · ${esc(r.note)}</p></div>`).join('');
    const td=document.getElementById('paper-todos'); if(td) td.textContent=(paper.todos_live||[]).join(' · ');
  }
  const roster=document.getElementById('roster-grid');
  if(roster){
    roster.innerHTML=(briefing.roster||[]).map(r=>`<div class="roster-card"><b>${esc(r.who)}</b><p>${esc(r.doing)}</p><small>${esc(r.where)}</small></div>`).join('');
  }
}

export function renderOverview(data){const cases=data.cases,formal=cases.filter(c=>c.metrics.formal_complete).length,dev=cases.reduce((a,c)=>a+c.metrics.development_pass_count,0),tasks=cases.reduce((a,c)=>a+c.metrics.task_count,0),tracks=cases.reduce((a,c)=>a+c.tracks.filter(t=>t.type==='physics').length,0);$('counts').innerHTML=kpi('正式整机验收',formal,'/ '+cases.length,'依据主流程最终结果')+kpi('完整开发试算通过',dev,'/ '+tasks+' 任务','插销试算与正式成绩分开显示')+kpi('真实运动记录',tracks,'条','保存前缀与完整运行均有范围标注')+kpi('本次新增预算','15','分钟 / 台','保留原余额与前一轮结果');$('timing-table').innerHTML=cases.map(c=>{const m=c.metrics;return `<tr><td><a href="?case=${encodeURIComponent(c.id)}">${esc(c.title)}</a><small>${esc(c.revision)} · ${esc(m.physical_parts)} 件</small></td>${m.stages.map(r=>`<td>${duration(r.active_seconds)}<small>wall ${duration(r.controller_wall_seconds)}</small>${r.carried_remaining_seconds>0.1?`<small>含原余额 ${duration(r.carried_remaining_seconds)}</small>`:""}</td>`).join('')}<td>${duration(m.stages.reduce((s,r)=>s+(r.active_seconds??0),0))}</td><td>${duration(m.controller_wall_total_seconds)}<small>停机间隔另计 ${duration(m.noncontroller_gap_seconds)}</small></td><td>${m.formal_passed}/${m.task_count}<small>${esc(m.development_passed?'另有完整开发试算通过':'尚无完整开发通过')}</small></td></tr>`}).join('');$('changes-overview').innerHTML=cases.map(c=>`<div class="change-row"><b>${esc(c.title)}</b><p>${esc(c.metrics.change_summary)}</p></div>`).join('')}
export function renderCaseSummary(c){const m=c.metrics;$('download-links').innerHTML=(c.downloads??[]).map(x=>`<a href="${esc(x.url)}" target="_blank">${esc(x.title)} ↗</a>`).join('');$('detail-metrics').innerHTML=[['实物零件 / 仿真刚体',`${m.physical_parts} / ${m.body_count}`],['正式任务通过',`${m.formal_passed} / ${m.task_count}`],['完整开发试算通过',String(m.development_pass_count)],['本次 active',duration(m.stages.at(-1)?.active_seconds)],['STEP 当前登记','已重新核验'],['Token 用量','记录不完整']].map(([label,value])=>`<div>${esc(label)}<b>${esc(value)}</b></div>`).join('');$('detail-tests').innerHTML=c.failure_display?failureTasks(c):m.test_rows.map(r=>`<div class="test-row"><b>${esc(r.title)}</b><p>${esc(r.status)}</p></div>`).join('');$('detail-iterations').innerHTML='<h3>这轮实际做了什么</h3>'+m.iteration_rows.map((s,i)=>`<div class="iteration"><span class="n">${String(i+1).padStart(2,'0')}</span><p>${esc(s)}</p></div>`).join('');$('detail-timing').innerHTML=m.stages.map(r=>`<div class="mini-stage"><span>${esc(r.title)}<br><small>wall ${duration(r.controller_wall_seconds)}</small>${r.carried_remaining_seconds>0.1?`<br><small>含原余额 ${duration(r.carried_remaining_seconds)}</small>`:""}</span><b>${duration(r.active_seconds)}</b></div>`).join('')+`<p class="footnote">跨轮累计 wall：${duration(m.cumulative_wall_seconds)}，含 ${duration(m.noncontroller_gap_seconds)} 停机间隔。各轮控制器运行 wall 合计：${duration(m.controller_wall_total_seconds)}。</p>`;$('detail-changes').innerHTML=`<p>${esc(m.change_summary)}</p>`+(m.change_details??[]).map(s=>`<p class="footnote">${esc(s)}</p>`).join('');$('detail-source').textContent=JSON.stringify({formal_result:m.formal_result_path,timing_source:m.timing_source,step_hotfix:m.step_hotfix,evidence:m.evidence_paths},null,2)}
