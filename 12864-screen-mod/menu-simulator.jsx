import React, { useState, useEffect, useCallback, useRef } from "react";

const FG      = "#c8e6c9";
const HL_BG   = "#c8e6c9";
const HL_FG   = "#0a1628";
const DIM     = "#4a7c59";
const TITLE   = "#90caf9";
const AMBER   = "#ffb300";
const RED     = "#ef5350";
const BLUE    = "#42a5f5";
const WHITE   = "#e0e0e0";
const GREEN   = "#66bb6a";

const SLOTS = [
  { valid:true,  label:"S1:3 stops,6s,loop" },
  { valid:false, label:"S2:Empty" },
  { valid:true,  label:"S3:10 stops,30s,loop" },
  { valid:true,  label:"S4:1 stop,0s,no loop" },
];

const MENUS = {
  ROOT:{
    title:"Main Menu",
    items:[
      {label:"Home All Motors",    action:"screen:HOMING"},
      {label:"Start / Stop",       action:"screen:START"},
      {label:"Select Program",     action:"menu:SELECT_PROGRAM"},
      {label:"Clear Program",      action:"menu:CLEAR_PROGRAM"},
      {label:"Full Loop Settings", action:"menu:FULL_LOOP"},
      {label:"Motor Control",      action:"menu:MOTOR_CONTROL"},
      {label:"Program Movement",   action:"menu:PROG_MOVEMENT"},
    ],
    parent:null, statusBar:true,
  },
  SELECT_PROGRAM:{
    title:"Select Program",
    items:[
      {label:"Full Loop",          action:"selectprog:Full Loop"},
      {label:"Move to Endpoint",   action:"selectprog:To Endpoint"},
      {label:SLOTS[0].label,       action:"selectprog:Slot 1"},
      {label:SLOTS[1].label,       action:"selectprog:Slot 2"},
      {label:SLOTS[2].label,       action:"selectprog:Slot 3"},
      {label:SLOTS[3].label,       action:"selectprog:Slot 4"},
      {label:"< Back",             action:"back"},
    ],
    parent:"ROOT",
  },
  CLEAR_PROGRAM:{
    title:"Clear Program",
    items:[
      {label:SLOTS[0].label,  action:"confirm:CLEAR:Clear Slot 1?"},
      {label:SLOTS[1].label,  action:"screen:EMPTY_SLOT"},
      {label:SLOTS[2].label,  action:"confirm:CLEAR:Clear Slot 3?"},
      {label:SLOTS[3].label,  action:"confirm:CLEAR:Clear Slot 4?"},
      {label:"Clear All",     action:"confirm:CLEARALL:Clear ALL slots?"},
      {label:"< Back",        action:"back"},
    ],
    parent:"ROOT",
  },
  FULL_LOOP:{
    title:"Full Loop Settings",
    items:[
      {label:"Set Speed",        action:"value:Full Loop Speed (mm/s):10"},
      {label:"Set Loop Count",   action:"value:Loop Count (0=cont):0"},
      {label:"Reset to Default", action:"screen:RESET_DONE"},
      {label:"< Back",           action:"back"},
    ],
    parent:"ROOT",
  },
  MOTOR_CONTROL:{
    title:"Motor Control",
    items:[
      {label:"Home All Motors",  action:"screen:HOMING"},
      {label:"Disable Motors",   action:"confirm:DISABLE:Disable motors?"},
      {label:"< Back",           action:"back"},
    ],
    parent:"ROOT",
  },
  PROG_MOVEMENT:{
    title:"Program Movement",
    items:[
      {label:"Select Slot",    action:"menu:PROG_SLOT"},
      {label:"Home",           action:"screen:HOMING"},
      {label:"Set Speed",      action:"value:Program Speed (mm/s):10"},
      {label:"Set Start",      action:"screen:POS_SAVED"},
      {label:"Add Stop Point", action:"addstop"},
      {label:"Set End Point",  action:"screen:POS_SAVED"},
      {label:"Set Loop Count", action:"value:Loop Count (0=cont):0"},
      {label:"Save Program",   action:"screen:SAVED"},
      {label:"< Back",         action:"back"},
    ],
    parent:"ROOT", jogMode:true,
  },
  PROG_SLOT:{
    title:"Select Slot",
    items:[
      {label:"Slot 1", action:"screen:SLOT_SEL"},
      {label:"Slot 2", action:"screen:SLOT_SEL"},
      {label:"Slot 3", action:"screen:SLOT_SEL"},
      {label:"Slot 4", action:"screen:SLOT_SEL"},
      {label:"< Back", action:"back"},
    ],
    parent:"PROG_MOVEMENT",
  },
};

const CONFIRMS = {
  CLEAR:        {sub:"Cannot be undone"},
  CLEARALL:     {sub:"Cannot be undone"},
  DISABLE:      {sub:"Carriage may move"},
  HOME_REQUIRED:{title:"Home required", sub:"Home before running?"},
};

const SCREENS = {
  HOMING:    {type:"info", msg:"Homing...",        sub:"Moving to left endstop", color:WHITE},
  RESET_DONE:{type:"info", msg:"Reset to default", sub:"Speed:10mm/s  Loop:cont", color:GREEN},
  POS_SAVED: {type:"info", msg:"Position saved",   sub:"", color:GREEN},
  SAVED:     {type:"info", msg:"Program saved",    sub:"Slot written OK", color:GREEN},
  SLOT_SEL:  {type:"info", msg:"Slot selected",    sub:"Set start/stops/end", color:GREEN},
  EMPTY_SLOT:{type:"info", msg:"Slot is empty",    sub:"Nothing to clear", color:AMBER},
  STOP_ADDED:{type:"info", msg:"Stop point added", sub:"Jog to next position", color:GREEN},
};

// ── Tiny LCD primitives ──────────────────────────────────────────────────────
function LCD({children,backlight}){
  const bg={idle:"#001800",running:"#00001a",paused:"#1a1000",estop:"#1a0000",homing:"#0d0d0d"}[backlight]||"#001800";
  return(
    <div style={{background:bg,border:"2px solid #2a2a2a",borderRadius:4,
      padding:"6px 8px",width:256,height:128,fontFamily:"'Courier New',monospace",
      overflow:"hidden",display:"flex",flexDirection:"column"}}>
      {children}
    </div>
  );
}
function Row({text,highlight,dim,color}){
  return(
    <div style={{background:highlight?HL_BG:"transparent",
      color:highlight?HL_FG:(color||FG),padding:"0 2px",
      fontSize:10,lineHeight:"13px",whiteSpace:"pre",
      opacity:dim?0.38:1,overflow:"hidden",maxWidth:"100%",flexShrink:0}}>
      {text}
    </div>
  );
}
function Div(){return <div style={{borderTop:`1px solid ${DIM}`,margin:"1px 0",flexShrink:0}}/>;}

// ── Main component ───────────────────────────────────────────────────────────
export default function MenuSim(){
  const [stack,setStack]           = useState([{id:"ROOT",cursor:0,scroll:0}]);
  const [activeProgram,setActiveProgram] = useState("None");
  const [backlight,setBacklight]   = useState("idle");
  const [valueState,setValueState] = useState(null);   // {label,value}
  const [confirmState,setConfirmState] = useState(null); // {id,title,cursor}
  const [screen,setScreen]         = useState(null);   // special screen id
  const [isHomed,setIsHomed]       = useState(false);
  const [runPhase,setRunPhase]     = useState("idle"); // idle/running/paused
  const [pos,setPos]               = useState(0);
  const [posDir,setPosDir]         = useState(1);
  const [stopCount,setStopCount]   = useState(0);
  const [pauseCursor,setPauseCursor] = useState(0); // 0=Resume 1=Cancel
  const [pendingLaunch,setPendingLaunch] = useState(false);
  const infoTimer                  = useRef(null);
  const VISIBLE = 4;

  // ── Simulated carriage position (bounces 0↔200) ─────────────────────
  useEffect(()=>{
    if(runPhase==="running"){
      const t=setInterval(()=>{
        setPos(p=>{
          const n=p+posDir*3;
          if(n>=200){setPosDir(-1);return 200;}
          if(n<=0)  {setPosDir(+1);return 0;}
          return n;
        });
      },100);
      return ()=>clearInterval(t);
    }
  },[runPhase,posDir]);

  // ── Auto-dismiss info screens ────────────────────────────────────────
  useEffect(()=>{
    if(screen && SCREENS[screen]?.type==="info"){
      if(infoTimer.current) clearTimeout(infoTimer.current);
      infoTimer.current=setTimeout(()=>{
        if(screen==="HOMING"){
          setIsHomed(true);
          if(pendingLaunch){
            setPendingLaunch(false);
            setRunPhase("running"); setBacklight("running"); setScreen("RUN");
            return;
          }
        }
        setScreen(null);
      },1600);
    }
    return ()=>{if(infoTimer.current) clearTimeout(infoTimer.current);};
  },[screen,pendingLaunch]);

  const top   = stack[stack.length-1];
  const menu  = MENUS[top?.id];
  const jogMode = !!menu?.jogMode;

  // ── Navigation ───────────────────────────────────────────────────────
  const goBack=useCallback(()=>{
    if(valueState)   {setValueState(null); return;}
    if(confirmState) {setConfirmState(null); return;}
    if(screen){
      if(screen==="RUN"||screen==="PAUSE"){setRunPhase("idle");setBacklight("idle");}
      setScreen(null); return;
    }
    if(stack.length>1) setStack(s=>s.slice(0,-1));
  },[stack,valueState,confirmState,screen]);

  const scrollMenu=useCallback((dir)=>{
    if(valueState){setValueState(v=>({...v,value:Math.max(0,Math.min(9999,v.value+dir))})); return;}
    if(confirmState){setConfirmState(c=>({...c,cursor:c.cursor===0?1:0})); return;}
    // Pause screen has its own 2-item cursor
    if(screen==="PAUSE"){setPauseCursor(c=>c===0?1:0); return;}
    if(screen) return;
    setStack(s=>{
      const t={...s[s.length-1]};
      const m=MENUS[t.id]; if(!m) return s;
      const n=m.items.length;
      t.cursor=Math.max(0,Math.min(n-1,t.cursor+dir));
      if(t.cursor<t.scroll) t.scroll=t.cursor;
      if(t.cursor>=t.scroll+VISIBLE) t.scroll=t.cursor-VISIBLE+1;
      return [...s.slice(0,-1),t];
    });
  },[valueState,confirmState,screen]);

  const doAddStop=useCallback(()=>{
    setValueState({label:"Stop Timer (sec)",value:3,isStop:true});
  },[]);

  const doLaunch=useCallback(()=>{
    setRunPhase("running"); setBacklight("running"); setScreen("RUN");
  },[]);

  const doHome=useCallback((thenLaunch=false)=>{
    setPendingLaunch(thenLaunch);
    setScreen("HOMING");
    setBacklight("homing");
  },[]);

  const select=useCallback(()=>{
    // Value editor
    if(valueState){
      if(valueState.isStop) setStopCount(c=>c+1);
      setValueState(null);
      return;
    }
    // Confirm screen
    if(confirmState){
      const yes=confirmState.cursor===0;
      if(yes && confirmState.id==="HOME_REQUIRED"){
        doHome(true);
      }
      setConfirmState(null);
      return;
    }
    // Special screens
    if(screen==="RUN")   {
      // Encoder press while running — pause and show pause screen with cursor at 0
      setPauseCursor(0);
      setRunPhase("paused"); setBacklight("paused"); setScreen("PAUSE"); return;
    }
    if(screen==="PAUSE") {
      if(pauseCursor===0){setRunPhase("running");setBacklight("running");setScreen("RUN");}
      else               {setRunPhase("idle");   setBacklight("idle");   setScreen(null);}
      return;
    }
    if(screen==="ESTOP") {setBacklight("idle"); setScreen(null); return;}
    if(screen)           {setScreen(null); return;}

    // Menu item
    if(!menu) return;
    const item=menu.items[top.cursor];
    if(!item) return;
    const action=item.action;

    if(action==="back")     {goBack(); return;}
    if(action==="addstop")  {doAddStop(); return;}

    if(action.startsWith("menu:")){
      setStack(s=>[...s,{id:action.slice(5),cursor:0,scroll:0}]); return;
    }
    if(action.startsWith("selectprog:")){
      setActiveProgram(action.slice(11));
      setStack(s=>[s[0]]); return;
    }
    if(action.startsWith("screen:")){
      const id=action.slice(7);
      if(id==="START"){
        if(!isHomed){setConfirmState({id:"HOME_REQUIRED",cursor:0}); return;}
        doLaunch(); return;
      }
      if(id==="HOMING"){doHome(false); return;}
      setScreen(id); return;
    }
    if(action.startsWith("confirm:")){
      const parts=action.split(":");
      setConfirmState({id:parts[1],title:parts[2]||"",cursor:0}); return;
    }
    if(action.startsWith("value:")){
      const parts=action.split(":");
      setValueState({label:parts[1],value:parseInt(parts[2])||0}); return;
    }
  },[valueState,confirmState,screen,menu,top,isHomed,goBack,doAddStop,doHome,doLaunch]);

  const triggerEstop=useCallback(()=>{
    setRunPhase("idle"); setBacklight("estop");
    setValueState(null); setConfirmState(null);
    setIsHomed(false); setScreen("ESTOP");
  },[]);

  const playPause=useCallback(()=>{
    // In Program Movement — add stop point shortcut
    if(jogMode){doAddStop(); return;}
    if(runPhase==="running"){setPauseCursor(0);setRunPhase("paused");setBacklight("paused");setScreen("PAUSE"); return;}
    if(runPhase==="paused") {setRunPhase("running");setBacklight("running");setScreen("RUN"); return;}
    // Start
    if(!isHomed){setConfirmState({id:"HOME_REQUIRED",cursor:0}); return;}
    doLaunch();
  },[jogMode,runPhase,isHomed,doAddStop,doLaunch]);

  // ── Keyboard ─────────────────────────────────────────────────────────
  useEffect(()=>{
    const h=e=>{
      if(e.key==="ArrowUp")                           {e.preventDefault();scrollMenu(-1);}
      if(e.key==="ArrowDown")                         {e.preventDefault();scrollMenu(+1);}
      if(e.key==="ArrowLeft"  && jogMode)             {e.preventDefault();setPos(p=>Math.max(0,p-8));}
      if(e.key==="ArrowRight" && jogMode)             {e.preventDefault();setPos(p=>Math.min(200,p+8));}
      if(e.key==="Enter"||e.key===" ")                {e.preventDefault();select();}
      if(e.key==="Escape"||e.key==="Backspace")       {e.preventDefault();goBack();}
      if(e.key==="e"||e.key==="E")                    triggerEstop();
      if(e.key==="p"||e.key==="P")                    playPause();
    };
    window.addEventListener("keydown",h);
    return ()=>window.removeEventListener("keydown",h);
  },[scrollMenu,select,goBack,triggerEstop,playPause,jogMode]);

  // ── LCD render ───────────────────────────────────────────────────────
  function renderLCD(){
    // E-Stop
    if(screen==="ESTOP") return(<>
      <Row text=""/><Row text=" !! E-STOP !!" color={RED}/><Row text=""/>
      <Div/><Row text=" Motors disabled" color={RED}/>
      <Row text=" Position unknown" dim/>
      <Row text=" ENC = Main Menu" dim/>
    </>);

    // Home required
    if(confirmState?.id==="HOME_REQUIRED"){
      const opts=["Yes — home now","No  — cancel"];
      return(<>
        <Row text="Home required" color={AMBER}/><Div/>
        <Row text="Motor not homed." color={WHITE}/>
        <Row text="Home before running?" dim/><Div/>
        <Row text={` ${opts[0]}`} highlight={confirmState.cursor===0}/>
        <Row text={` ${opts[1]}`} highlight={confirmState.cursor===1}/>
      </>);
    }

    // Run screen
    if(screen==="RUN") return(<>
      <Row text="** RUNNING **" color={BLUE}/><Div/>
      <Row text={`Prog: ${activeProgram.substring(0,17)}`}/>
      <Row text={`Pos:  ${String(pos).padStart(4)} mm`}/>
      <Row text={`Dir:  ${posDir>0?">>> (fwd)":"<<< (rev)"}`} color={posDir>0?GREEN:AMBER}/>
      <Div/><Row text="PLAY=Pause  ENC=EStop" dim/>
    </>);

    // Pause screen
    if(screen==="PAUSE"){
      const opts=["Resume","Cancel Program"];
      return(<>
        <Row text="   PAUSED" color={AMBER}/>
        <Row text={`Pos: ${String(pos).padStart(4)} mm`} dim/><Div/>
        <Row text={` ${opts[0]}`} highlight={pauseCursor===0}/>
        <Row text={` ${opts[1]}`} highlight={pauseCursor===1}/>
        <Div/><Row text="Turn=select  ENC=OK" dim/>
      </>);
    }

    // Info screens
    if(screen && SCREENS[screen]?.type==="info"){
      const s=SCREENS[screen];
      return(<>
        <Row text=""/>
        <Row text={`  ${s.msg}`} color={s.color||GREEN}/>
        {s.sub?<Row text={`  ${s.sub}`} dim/>:null}
        {screen==="HOMING"?<Row text="  <<<<<<<<<<<" color={AMBER}/>:null}
        <Row text=""/><Row text="  (returning...)" dim/>
      </>);
    }

    // Value editor
    if(valueState){
      const v=String(valueState.value).padStart(5);
      return(<>
        <Row text={valueState.label.substring(0,21)} color={TITLE}/><Div/>
        <Row text=""/><Row text={`       ${v}`} color={WHITE}/>
        <Row text=""/><Div/>
        <Row text="Turn=adjust  ENC=OK" dim/>
      </>);
    }

    // Confirm
    if(confirmState){
      const cfg=CONFIRMS[confirmState.id]||{};
      const title=confirmState.title||cfg.title||"Confirm?";
      return(<>
        <Row text={title.substring(0,21)} color={AMBER}/>
        <Row text={(cfg.sub||"").substring(0,21)} dim/><Div/>
        <Row text=" Yes — confirm" highlight={confirmState.cursor===0}/>
        <Row text=" No  — cancel"  highlight={confirmState.cursor===1}/>
        <Div/><Row text="Turn=select  ENC=OK" dim/>
      </>);
    }

    // Regular menu
    if(!menu) return null;
    const rows=[];
    rows.push(<Row key="t" text={menu.title} color={TITLE}/>);
    rows.push(<Div key="d"/>);
    for(let i=0;i<VISIBLE;i++){
      const idx=top.scroll+i;
      if(idx>=menu.items.length){rows.push(<Row key={`e${i}`} text=""/>);continue;}
      const item=menu.items[idx];
      const isBack=item.label==="< Back";
      rows.push(<Row key={idx}
        text={` ${item.label.substring(0,19)}`}
        highlight={idx===top.cursor}
        dim={isBack} color={isBack?DIM:undefined}/>);
    }
    if(menu.jogMode){
      rows.push(<Div key="jd"/>);
      rows.push(<Row key="jh" text={`Stops:${stopCount} Pos:${String(pos).padStart(3)}mm PLAY=add`} dim color={AMBER}/>);
    }
    if(menu.statusBar){
      rows.push(<Div key="sd"/>);
      const homedIndicator=isHomed?" H":" !";
      rows.push(<Row key="sb" text={`Pgm:${activeProgram.substring(0,14)}${homedIndicator}`} dim color={AMBER}/>);
    }
    return rows;
  }

  // ── Backlight LEDs ───────────────────────────────────────────────────
  const blCol={
    idle:   ["#00cc00","#008800"],
    running:["#0055ff","#002299"],
    paused: ["#ffaa00","#885500"],
    estop:  ["#ff2200","#881100"],
    homing: ["#cccccc","#666666"],
  };
  const [blA,blB]=blCol[backlight]||blCol.idle;

  const btn=(bg,onClick,label,sub)=>(
    <div style={{textAlign:"center"}}>
      <button onClick={onClick} style={{background:bg,color:"#fff",border:"none",
        borderRadius:5,padding:"7px 12px",cursor:"pointer",fontSize:11,
        fontFamily:"Arial",userSelect:"none"}}>{label}</button>
      {sub&&<div style={{color:"#444",fontSize:9,marginTop:2}}>{sub}</div>}
    </div>
  );

  const breadcrumb=stack.map(s=>MENUS[s.id]?.title||s.id).join(" › ");

  return(
    <div style={{minHeight:"100vh",background:"#0f172a",display:"flex",
      flexDirection:"column",alignItems:"center",justifyContent:"center",
      gap:20,padding:24,fontFamily:"Arial,sans-serif"}}>

      <div style={{color:TITLE,fontSize:17,fontWeight:"bold",letterSpacing:1}}>
        Camera Slider — 12864 Mod — Menu Simulator
      </div>

      <div style={{background:"#1c1c1c",borderRadius:10,padding:18,
        boxShadow:"0 8px 40px #000c",display:"flex",
        flexDirection:"column",alignItems:"center",gap:12}}>

        {/* NeoPixel LEDs + status */}
        <div style={{display:"flex",gap:8,alignSelf:"flex-start",marginLeft:4,alignItems:"center"}}>
          {[blA,blB].map((c,i)=>(
            <div key={i} style={{width:10,height:10,borderRadius:"50%",
              background:c,boxShadow:`0 0 8px ${c}`}}/>
          ))}
          <span style={{color:"#555",fontSize:10,marginLeft:6}}>
            <span style={{color:blA}}>{backlight}</span>
            {!isHomed&&<span style={{color:RED,marginLeft:10}}>⚠ not homed</span>}
            {isHomed &&<span style={{color:GREEN,marginLeft:10}}>✓ homed</span>}
            {jogMode &&<span style={{color:AMBER,marginLeft:10}}>jog mode active</span>}
          </span>
        </div>

        {/* LCD */}
        <LCD backlight={backlight}>{renderLCD()}</LCD>

        {/* Encoder wheel */}
        <div style={{display:"flex",alignItems:"center",gap:14}}>
          {btn("#0f3460",()=>scrollMenu(-1),"▲")}
          <div style={{textAlign:"center"}}>
            <div style={{width:46,height:46,borderRadius:"50%",
              background:"#2a2a2a",border:"3px solid #555",
              display:"flex",alignItems:"center",justifyContent:"center",
              cursor:"pointer",color:"#aaa",fontSize:9}}
              onClick={select}
              onContextMenu={e=>{e.preventDefault();triggerEstop();}}
              title="Click=OK  Right-click=E-Stop">ENC</div>
            <div style={{color:"#444",fontSize:9,marginTop:2}}>click=OK<br/>r-click=EStop</div>
          </div>
          {btn("#0f3460",()=>scrollMenu(+1),"▼")}
        </div>

        {/* External buttons row */}
        <div style={{display:"flex",gap:8,flexWrap:"wrap",justifyContent:"center"}}>
          {btn("#1b5e20",()=>setPos(p=>Math.max(0,p-8)),"◀ LEFT","jog left")}
          {btn("#4a148c",playPause,
            jogMode?"➕ STOP PT":runPhase==="running"?"⏸ PAUSE":runPhase==="paused"?"▶ RESUME":"▶ PLAY",
            jogMode?"adds stop point":"play / pause")}
          {btn("#1b5e20",()=>setPos(p=>Math.min(200,p+8)),"RIGHT ▶","jog right")}
          {btn("#b71c1c",triggerEstop,"⛔ E-STOP","enc long press")}
        </div>

        {/* Simulate home helper */}
        <button onClick={()=>doHome(false)}
          style={{background:"#374151",color:"#ccc",border:"none",
            borderRadius:4,padding:"5px 14px",cursor:"pointer",fontSize:10}}>
          🏠 Simulate Home
        </button>
      </div>

      {/* Breadcrumb */}
      <div style={{color:"#4a7c59",fontSize:11}}>
        {valueState?`${breadcrumb} › [Edit: ${valueState.label}]`
          :confirmState?`${breadcrumb} › [${confirmState.id}]`
          :screen?`${breadcrumb} › [${screen}]`
          :breadcrumb}
      </div>

      {/* Key hints */}
      <div style={{color:"#374151",fontSize:10,textAlign:"center",lineHeight:2}}>
        ↑↓ scroll &nbsp;|&nbsp; Enter/Space = select &nbsp;|&nbsp; Esc = back
        &nbsp;|&nbsp; E = E-Stop &nbsp;|&nbsp; P = Play/Pause<br/>
        ←→ = jog carriage (when in Program Movement)
        &nbsp;|&nbsp; Right-click encoder = E-Stop
      </div>
    </div>
  );
}
