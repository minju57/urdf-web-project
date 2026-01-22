import streamlit as st
import math

# 페이지 설정
st.set_page_config(page_title="URDF Generator", layout="wide")

st.title("🤖 Web-based URDF Generator")
st.markdown("조인트를 추가하고 설정한 뒤 URDF 파일을 다운로드하세요.")

# 세션 상태 초기화 (Joint 데이터를 저장할 리스트)
if 'joints' not in st.session_state:
    st.session_state.joints = [{'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'r': 0.0, 'p': 0.0, 'yaw': 0.0, 'low': -3.14, 'up': 3.14}]

# 버튼: 조인트 추가/삭제
col_btns = st.columns([1, 1, 5])
with col_btns[0]:
    if st.button("➕ Add Joint"):
        st.session_state.joints.append({'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'r': 0.0, 'p': 0.0, 'yaw': 0.0, 'low': -3.14, 'up': 3.14})
with col_btns[1]:
    if st.button("🗑️ Reset"):
        st.session_state.joints = []
        st.rerun()

# 입력 화면 구성 (가로로 배열된 카드 형태)
joint_data_list = []
cols = st.columns(len(st.session_state.joints) if len(st.session_state.joints) > 0 else 1)

for i, joint in enumerate(st.session_state.joints):
    with cols[i]:
        with st.expander(f"Joint {i}", expanded=True):
            axis = st.selectbox(f"Axis##{i}", ["Roll", "Pitch", "Yaw"], index=["Roll", "Pitch", "Yaw"].index(joint['axis']))
            
            st.write("**Offset (m, rad)**")
            x = st.number_input(f"x##{i}", value=joint['x'], step=0.1)
            y = st.number_input(f"y##{i}", value=joint['y'], step=0.1)
            z = st.number_input(f"z##{i}", value=joint['z'], step=0.1)
            
            st.write("**Rotation**")
            r = st.number_input(f"r##{i}", value=joint['r'], step=0.1)
            p = st.number_input(f"p##{i}", value=joint['p'], step=0.1)
            yaw = st.number_input(f"yaw##{i}", value=joint['yaw'], step=0.1)
            
            st.write("**Limits**")
            low = st.number_input(f"Lower##{i}", value=joint['low'], step=0.1)
            up = st.number_input(f"Upper##{i}", value=joint['up'], step=0.1)
            
            joint_data_list.append({
                'axis': axis, 'x': x, 'y': y, 'z': z, 
                'r': r, 'p': p, 'yaw': yaw, 'low': low, 'up': up
            })

# URDF 생성 로직
def generate_urdf(data):
    xml = ['<?xml version="1.0"?>', '<robot name="web_robot">', '  <link name="world"/>']
    axis_map = {"Roll": "1 0 0", "Pitch": "0 1 0", "Yaw": "0 0 1"}
    
    for i, j in enumerate(data):
        parent = "world" if i == 0 else f"link_{i-1}"
        xml.append(f'  <joint name="joint_{i}" type="revolute">')
        xml.append(f'    <parent link="{parent}"/><child link="link_{i}"/>')
        xml.append(f'    <origin xyz="{j["x"]} {j["y"]} {j["z"]}" rpy="{j["r"]} {j["p"]} {j["yaw"]}"/>')
        xml.append(f'    <axis xyz="{axis_map[j["axis"]]}"/>')
        xml.append(f'    <limit lower="{j["low"]}" upper="{j["up"]}" effort="10" velocity="1"/>')
        xml.append('  </joint>')
        xml.append(f'  <link name="link_{i}"><visual><geometry><cylinder radius="0.05" length="0.2"/></geometry></visual></link>')
    
    xml.append('</robot>')
    return "\n".join(xml)

# 결과 출력 및 다운로드
st.divider()
final_urdf = generate_urdf(joint_data_list)

st.subheader("Generated URDF Preview")
st.code(final_urdf, language='xml')

st.download_button(
    label="💾 Download URDF File",
    data=final_urdf,
    file_name="robot.urdf",
    mime="text/xml"
)import streamlit as st
import pyvista as pv
from st_pyvista_viewer import st_pyvista_viewer
import numpy as np

# 페이지 설정 (레이아웃을 크게 쓰기 위해 wide 모드)
st.set_page_config(page_title="URDF Generator & Visualizer", layout="wide")

# --- CSS: 카드 형태의 깔끔한 디자인 적용 ---
st.markdown("""
    <style>
    .stExpander { border: 1px solid #e6e9ef; border-radius: 10px; background-color: #f9f9f9; }
    </style>
    """, unsafe_allow_html=True)

st.title("🤖 Web-based URDF Generator & Visualizer")

# 세션 상태 초기화 (Joint 데이터를 저장할 리스트)
if 'joints' not in st.session_state:
    st.session_state.joints = [{'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'r': 0.0, 'p': 0.0, 'yaw': 0.0, 'low': -3.14, 'up': 3.14}]

# 화면 레이아웃 분할: 왼쪽(입력창 및 코드) | 오른쪽(3D 시각화)
main_col_left, main_col_right = st.columns([1, 1])

with main_col_left:
    st.subheader("🛠️ Joint Configuration")
    # 버튼: 조인트 추가/삭제
    col_btns = st.columns([1, 1, 3])
    with col_btns[0]:
        if st.button("➕ Add Joint"):
            st.session_state.joints.append({'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'r': 0.0, 'p': 0.0, 'yaw': 0.0, 'low': -3.14, 'up': 3.14})
    with col_btns[1]:
        if st.button("🗑️ Reset"):
            st.session_state.joints = [{'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'r': 0.0, 'p': 0.0, 'yaw': 0.0, 'low': -3.14, 'up': 3.14}]
            st.rerun()

    # 입력 화면 구성 (세로로 정렬된 조인트 카드)
    joint_data_list = []
    for i, joint in enumerate(st.session_state.joints):
        with st.expander(f"Joint {i} - {joint['axis']}", expanded=True):
            c1, c2, c3 = st.columns(3)
            with c1:
                axis = st.selectbox(f"Axis##{i}", ["Roll", "Pitch", "Yaw"], index=["Roll", "Pitch", "Yaw"].index(joint['axis']))
            with c2:
                low = st.number_input(f"Lower Rad##{i}", value=joint['low'], step=0.1)
            with c3:
                up = st.number_input(f"Upper Rad##{i}", value=joint['up'], step=0.1)
            
            st.write("---")
            c4, c5, c6 = st.columns(3)
            with c4:
                x = st.number_input(f"x (m)##{i}", value=joint['x'], step=0.1)
                r = st.number_input(f"r (rad)##{i}", value=joint['r'], step=0.1)
            with c5:
                y = st.number_input(f"y (m)##{i}", value=joint['y'], step=0.1)
                p = st.number_input(f"p (rad)##{i}", value=joint['p'], step=0.1)
            with c6:
                z = st.number_input(f"z (m)##{i}", value=joint['z'], step=0.1)
                yaw = st.number_input(f"yaw (rad)##{i}", value=joint['yaw'], step=0.1)
            
            joint_data_list.append({
                'axis': axis, 'x': x, 'y': y, 'z': z, 
                'r': r, 'p': p, 'yaw': yaw, 'low': low, 'up': up
            })
    
    # 조인트 데이터 업데이트
    st.session_state.joints = joint_data_list

with main_col_right:
    st.subheader("🏗️ 3D Visualizer")
    
    # --- PyVista 3D 렌더링 로직 ---
    plotter = pv.Plotter(window_size=[600, 600])
    plotter.set_background("#f0f2f6") # 깔끔한 배경색
    
    # 바닥면 (World plane) - C++의 world link 대응
    floor = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), i_size=2, j_size=2)
    plotter.add_mesh(floor, color="#bdc3c7", show_edges=True, opacity=0.5)
    plotter.add_axes()

    # 로봇 모델 그리기 (FK: Forward Kinematics 간이 구현)
    current_pos = np.array([0.0, 0.0, 0.0])
    for i, j in enumerate(joint_data_list):
        # 1. 조인트 오프셋만큼 이동
        current_pos += np.array([j['x'], j['y'], j['z']])
        
        # 2. 링크(실린더) 생성 - C++의 cylinder mesh 대응
        # 실린더 길이는 0.2m, 반지름은 0.05m
        cylinder = pv.Cylinder(center=(current_pos[0], current_pos[1], current_pos[2]), 
                               direction=(0, 0, 1), radius=0.05, height=0.2)
        
        # 3. 색상 결정 (C++의 짝수/홀수 색상 로직)
        color = "red" if i % 2 == 0 else "blue"
        plotter.add_mesh(cylinder, color=color, smooth_shading=True)
        
        # 4. 조인트 위치 표시 (작은 구)
        sphere = pv.Sphere(radius=0.03, center=current_pos)
        plotter.add_mesh(sphere, color="yellow")

    # 웹 화면에 출력
    st_pyvista_viewer(plotter, key="robot_viz")

# --- 하단: URDF 생성 코드 및 다운로드 ---
st.divider()
def generate_urdf(data):
    xml = ['<?xml version="1.0"?>', '<robot name="web_robot">', '  <link name="world"/>']
    axis_map = {"Roll": "1 0 0", "Pitch": "0 1 0", "Yaw": "0 0 1"}
    for i, j in enumerate(data):
        parent = "world" if i == 0 else f"link_{i-1}"
        xml.append(f'  <joint name="joint_{i}" type="revolute">')
        xml.append(f'    <parent link="{parent}"/><child link="link_{i}"/>')
        xml.append(f'    <origin xyz="{j["x"]} {j["y"]} {j["z"]}" rpy="{j["r"]} {j["p"]} {j["yaw"]}"/>')
        xml.append(f'    <axis xyz="{axis_map[j["axis"]]}"/>')
        xml.append(f'    <limit lower="{j["low"]}" upper="{j["up"]}" effort="10" velocity="1"/>')
        xml.append('  </joint>')
        xml.append(f'  <link name="link_{i}"><visual><geometry><cylinder radius="0.05" length="0.2"/></geometry></visual></link>')
    xml.append('</robot>')
    return "\n".join(xml)

final_urdf = generate_urdf(joint_data_list)
st.subheader("📄 Generated URDF Code")
st.code(final_urdf, language='xml')

st.download_button(
    label="💾 Download URDF File",
    data=final_urdf,
    file_name="robot.urdf",
    mime="text/xml"
)