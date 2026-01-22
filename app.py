import streamlit as st
import pyvista as pv
import numpy as np

# 페이지 설정
st.set_page_config(page_title="URDF Generator", layout="wide")

st.title("🤖 Web-based URDF Generator & Visualizer")
st.markdown("조인트를 설정하면 우측에 3D 모델이 실시간으로 업데이트됩니다.")

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
        with st.expander(f"Joint {i}", expanded=True):
            c1, c2, c3 = st.columns(3)
            with c1:
                axis = st.selectbox(f"Axis##{i}", ["Roll", "Pitch", "Yaw"], 
                                   index=["Roll", "Pitch", "Yaw"].index(joint['axis']))
            with c2:
                low = st.number_input(f"Lower##{i}", value=joint['low'], step=0.1)
            with c3:
                up = st.number_input(f"Upper##{i}", value=joint['up'], step=0.1)
            
            c4, c5, c6 = st.columns(3)
            with c4: x = st.number_input(f"x (m)##{i}", value=joint['x'], step=0.1)
            with c5: y = st.number_input(f"y (m)##{i}", value=joint['y'], step=0.1)
            with c6: z = st.number_input(f"z (m)##{i}", value=joint['z'], step=0.1)
            
            joint_data_list.append({'axis': axis, 'x': x, 'y': y, 'z': z, 'low': low, 'up': up})

# 2. 오른쪽: 3D 시각화 및 URDF 미리보기
with right_col:
    st.subheader("🏗️ 3D Preview")
    
    # PyVista를 이용한 실시간 로봇 렌더링
    plotter = pv.Plotter(window_size=[600, 500], off_screen=True)
    plotter.set_background("#f0f2f6")
    
    # 바닥판 (Grid)
    floor = pv.Plane(i_size=2, j_size=2)
    plotter.add_mesh(floor, color="gray", opacity=0.2, show_edges=True)
    plotter.add_axes()

    # 순차적 위치 계산 (FK 간이 구현)
    current_pos = np.array([0.0, 0.0, 0.0])
    for i, j in enumerate(joint_data_list):
        # 조인트 위치 이동
        prev_pos = current_pos.copy()
        current_pos += np.array([j['x'], j['y'], j['z']])
        
        # 링크(원통) 생성
        length = 0.2
        cylinder = pv.Cylinder(center=(current_pos[0], current_pos[1], current_pos[2]), 
                               direction=(0, 0, 1), radius=0.04, height=length)
        
        # 색상 (빨강/파랑 교체)
        color = "red" if i % 2 == 0 else "blue"
        plotter.add_mesh(cylinder, color=color)
        
        # 조인트 구체 표현
        sphere = pv.Sphere(radius=0.04, center=current_pos)
        plotter.add_mesh(sphere, color="yellow")

    # HTML로 변환하여 화면에 표시
    html_content = plotter.export_html(None)
    st.components.v1.html(html_content, height=500)

# --- 하단: URDF 생성 결과 ---
st.divider()
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