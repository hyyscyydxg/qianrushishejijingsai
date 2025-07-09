/**
 * Copyright 2024 Beijing Volcano Engine Technology Co., Ltd. All Rights Reserved.
 * SPDX-license-identifier: BSD-3-Clause
 */

'use strict';

let rtc = new RtcClient({
  config,
  streamOptions,
  handleUserPublishStream,
  handleUserUnpublishStream,
  handleUserStartVideoCapture,
  handleUserStopVideoCapture,
  handleUserJoin,
  handleUserLeave,
  handleEventError,
});

const { roomId = '' } = getUrlArgs();
const hasLogin = checkLoginInfo();

if (!hasLogin) {
  $('.player').hide();                 // ★ 修改：登录前隐藏视频区域
  $('#control').hide();
  $('#log-btn').hide();                // ★ 新增：隐藏日志按钮
  $('#log-output').hide();             // ★ 新增：隐藏日志框
  $('#room-id-text').text('');
  $('#user-id-text').text('');
  $('#header-version').text(`RTC版本 v${rtc.SDKVERSION}`);
  $('#room-id').val(roomId);
  $('#pannel').show();

  fillRoomId();
  checkRoomIdAndUserId('room-id');
  checkRoomIdAndUserId('user-id');
} else {
  const { roomId, uid } = getSessionInfo();
  config.roomId = roomId;
  config.uid = uid;
  switchToMeeting(config);
}

const changeMicOrVideoIconUrl = (type, statusTag, offIconUrl, onIconUrl) => {
  let iconSrc = statusTag ? onIconUrl : offIconUrl;
  $(`#control-${type} img`).attr('src', iconSrc);
};

async function switchToMeeting(config) {
  try {
    rtc.bindEngineEvents();
    let token = null;
    config.tokens.forEach(it => { if (it.userId === config.uid) token = it.token; });

    await rtc.join(token, config.roomId, config.uid);
    console.log('join room');

    setSessionInfo({ roomId: config.roomId, uid: config.uid });

    $('#header-version').text(config.roomId);
    $('.player').show();          // ★ 显示视频
    $('#log-btn').show();         // ★ 显示日志按钮
    $('#log-output').show();      // ★ 显示日志框
    $('#pannel').hide();
    $('#join-res').hide().text('');
  } catch (err) {
    $('#control').hide();
    $('#pannel').show();
    $('.player').hide();
    $('#log-btn').hide();         // ★ 若失败也隐藏
    $('#log-output').hide();      // ★
    $('#join-res').show().text(JSON.stringify(err));
    console.log(err);
  }
}

$('#submit').on('click', async () => {
  if (checkReg('room-id') || checkReg('user-id')) return;

  config.roomId = $('#room-id').val();
  config.uid = $('#user-id').val();
  changeUrl(config.roomId);

  try {
    await fetch('http://localhost:5000/api/start_rtc', { method: 'POST' });
    console.log('已向后端请求启动 RTCTest 推流');
  } catch (err) {
    console.error('调用后端 /api/start_rtc 接口失败', err);
  }

  switchToMeeting(config);
});

$('#control-mic').click(() => actionChangeMicState());
$('#control-video').click(() => actionChangeVideoState());
$('#leave').click(() => actionToLeave());

window.addEventListener('pagehide', () => {
  rtc.removeEventListener();
  rtc.leave();
});

/*---------------------- action handler start --------------------*/
const actionChangeMicState = async () => {
  isMicOn = !isMicOn;
  await rtc.changeAudioState(isMicOn);
  changeMicOrVideoIconUrl('mic', isMicOn, OFFMICICON, ONMICICON);
};

const actionChangeVideoState = async () => {
  isVideoOn = !isVideoOn;
  await rtc.changeVideoState(isVideoOn);
  changeMicOrVideoIconUrl('video', isVideoOn, OFFVIDEOICON, ONVIDEOICON);
};

const actionToLeave = async () => {
  const { roomId = '' } = getUrlArgs();

  $('#header-version').text(`RTC版本 v${rtc.SDKVERSION}`);
  $('.remote-player').remove();
  $('#user-id').val('');
  $('#room-id').val(roomId);
  $('#control').hide();
  $('.player').hide();
  $('#log-btn').hide();           // ★ 离开时再次隐藏
  $('#log-output').hide();        // ★
  $('#pannel').show();

  isMicOn = true;
  changeMicOrVideoIconUrl('mic', isMicOn, OFFMICICON, ONMICICON);
  isVideoOn = true;
  changeMicOrVideoIconUrl('video', isVideoOn, OFFVIDEOICON, ONVIDEOICON);

  rtc.removeEventListener();
  removeLoginInfo();
  await rtc.leave();
};
/*---------------------- action handler end --------------------*/

/*------------------------- common handler start ----------------*/
function handleUserJoin(e) {
  const remoteUserId = e.userInfo.userId;
  if ($('.player-wrapper').length < 4) {
    $('#player-list').append(`
      <div id="player-wrapper-${remoteUserId}" class="player-wrapper remote-player">
        <p class="player-name">${remoteUserId}</p>
      </div>`);
  }
}

function handleUserLeave(e) {
  const remoteUserId = e.userInfo.userId;
  $(`#player-wrapper-${remoteUserId}`).remove();
  rtc.checkAutoPlayFailUser(remoteUserId);
}

async function handleUserPublishStream(stream) {
  const { userId, mediaType } = stream;
  if (mediaType & rtc.MediaType.VIDEO) {
    const player = $(`#player-wrapper-${userId}`);
    if (player[0]) rtc.setRemoteVideoPlayer(userId, player[0]);
  }
}

function handleUserUnpublishStream(stream) {
  const { userId, mediaType } = stream;
  if (mediaType & rtc.MediaType.VIDEO) rtc.setRemoteVideoPlayer(userId, undefined);
}

function handleEventError(e) {
  if (e.errorCode === VERTC.ErrorCode.DUPLICATE_LOGIN) {
    actionToLeave();
    alert('你的账号被其他人顶下线了');
    closeModal();
    rtc.autoPlayFailUser = [];
  }
}

async function handleUserStartVideoCapture(event) {
  const player = $(`#player-wrapper-${event.userId}`);
  if (player[0]) rtc.setRemoteVideoPlayer(event.userId, player[0]);
}

function handleUserStopVideoCapture(event) {
  rtc.setRemoteVideoPlayer(event.userId, undefined);
}

if (location.host.includes('boe')) new VConsole();

document.body.insertAdjacentHTML('beforeend', `
  <button id="log-btn" style="
    display: none;
    position: fixed;
    top: 20px;
    right: 180px;
    z-index: 9999;
    background-color: #1976d2;
    color: white;
    border: none;
    border-radius: 6px;
    padding: 8px 16px;
    font-size: 14px;
    cursor: pointer;
    box-shadow: 0 2px 6px rgba(0, 0, 0, 0.2);
  ">生成日志分析</button>

  <div id="log-output" style="
    display: none;
    position: fixed;
    top: 70px;
    right: 20px;
    width: 45vw;
    max-height: 80vh;
    overflow-y: auto;
    background: #ffffff;
    border-radius: 10px;
    box-shadow: 0 4px 18px rgba(0, 0, 0, 0.15);
    padding: 20px;
    white-space: pre-wrap;
    font-family: 'Courier New', monospace;
    font-size: 14px;
    line-height: 1.6;
    border: 1px solid #ddd;
    z-index: 9999;
  ">
    <b>日志输出：</b>
  </div>
`);

document.getElementById('log-btn').onclick = async () => {
  const output = document.getElementById('log-output');
  output.style.display = 'block';
  output.innerText = '⏳ 正在生成智能摘要，请稍候...\n';

  try {
    const resp = await fetch('http://localhost:5000/trigger_log', { method: 'POST' });
    const data = await resp.json();

    if (data.status === 'ok') {
      const summaryBlocks = data.summary
        .split(/\n\s*\n/) // 按双换行分段
        .map(block => `
          <div style="
            background:#f5f5f5;
            border-left: 4px solid #1976d2;
            padding: 12px 16px;
            margin-bottom: 12px;
            border-radius: 6px;
            white-space: normal;
            font-family: 'Courier New', monospace;
            font-size: 14px;
            line-height: 1.6;
            color: #333;
          ">
            ${block.replace(/\n/g, '<br>')}
          </div>
        `)
        .join('');

      output.innerHTML = `
        <h3 style="color:#2e7d32; margin-top:0; margin-bottom:16px;">✅ 智能日志总结</h3>
        ${summaryBlocks}
      `;
    } else {
      output.innerText = `❌ 错误: ${data.message}`;
    }
  } catch (err) {
    output.innerText = `❌ 网络或服务器错误: ${err}`;
  }
};

