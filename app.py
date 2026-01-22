import streamlit as st
import numpy as np

# 페이지 설정
st.set_page_config(page_title="URDF Generator", layout="wide")

st.title("🤖 URDF Generator (Light Version)")

# 세션 상태 초기화
if 'joints' not in st.session_state:
    st.session_state.joints = [{'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'low': -3.14, 'up': 3.14}]

# --- 상단 버튼 영역 ---
col_btns = st.columns([1, 1, 8])
with col_btns[0]:
    if st.button("➕ Add Joint"):
        st.session_state.joints.append({'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.2, 'low': -3.14, 'up': 3.14})
with col_btns[1]:
    if st.button("🗑️ Reset"):
        st.session_state.joints = [{'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'low': -3.14, 'up': 3.14}]
        st.rerun()

# --- 메인 레이아웃 분할 ---
left_col, right_col = st.columns([1, 1])

# 1. 왼쪽: 조인트 설정 입력창
joint_data_list = []
with left_col:
    st.subheader("🛠️ Joint Settings")
    for i, joint in enumerate(st.session_state.joints):
        with st.expander(f"Joint {i} - {joint['axis']}", expanded=True):
            c1, c2, c3 = st.columns(3)
            with c1:
                axis = st.selectbox(f"Axis##{i}", ["Roll", "Pitch", "Yaw"], 
                                   index=["Roll", "Pitch", "Yaw"].index(joint['axis']))
            with c2:
                low = st.number_input(f"Lower##{i}", value=joint['low'], key=f"low_{i}")
            with c3:
                up = st.number_input(f"Upper##{i}", value=joint['up'], key=f"up_{i}")
            
            c4, c5, c6 = st.columns(3)
            with c4: x = st.number_input(f"x (m)##{i}", value=joint['x'], step=0.1, key=f"x_{i}")
            with c5: y = st.number_input(f"y (m)##{i}", value=joint['y'], step=0.1, key=f"y_{i}")
            with c6: z = st.number_input(f"z (m)##{i}", value=joint['z'], step=0.1, key=f"z_{i}")
            
            joint_data_list.append({'axis': axis, 'x': x, 'y': y, 'z': z, 'low': low, 'up': up})

# 2. 오른쪽: 시각화 레이아웃(플레이스홀더) 및 URDF 미리보기
with right_col:
    st.subheader("🏗️ 3D Preview (Placeholder)")
    # 시각화 대신 회색 박스로 레이아웃만 표시
    st.container(border=True).write("🎨 3D 시각화 엔진 연결 대기 중... (현재 레이아웃 모드)")
    
    st.divider()
    
    # 실시간 URDF 코드 생성 결과
    def generate_urdf(data):
        xml = ['<?xml version="1.0"?>', '<robot name="web_robot">', '  <link name="world"/>']
        axis_map = {"Roll": "1 0 0", "Pitch": "0 1 0", "Yaw": "0 0 1"}
        for i, j in enumerate(data):
            parent = "world" if i == 0 else f"link_{i-1}"
            xml.append(f'  <joint name="joint_{i}" type="revolute">')
            xml.append(f'    <parent link="{parent}"/><child link="link_{i}"/>')
            xml.append(f'    <origin xyz="{j["x"]} {j["y"]} {j["z"]}" rpy="0 0 0"/>')
            xml.append(f'    <axis xyz="{axis_map[j["axis"]]}"/>')
            xml.append(f'    <limit lower="{j["low"]}" upper="{j["up"]}" effort="10" velocity="1"/>')
            xml.append('  </joint>')
            xml.append(f'  <link name="link_{i}"><visual><geometry><cylinder radius="0.05" length="0.2"/></geometry></visual></link>')
        xml.append('</robot>')
        return "\n".join(xml)

    final_urdf = generate_urdf(joint_data_list)
    st.subheader("📄 Generated URDF Code")
    st.code(final_urdf, language='xml')
    st.download_button(label="💾 Download URDF", data=final_urdf, file_name="robot.urdf", mime="text/xml")