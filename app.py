import streamlit as st

# 페이지 설정
st.set_page_config(page_title="URDF Generator", layout="wide")

st.title("🤖 Web-based URDF Generator")
st.markdown("조인트를 추가하고 설정한 뒤 URDF 파일을 다운로드하세요.")

# 세션 상태 초기화
if 'joints' not in st.session_state:
    st.session_state.joints = [{'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'r': 0.0, 'p': 0.0, 'yaw': 0.0, 'low': -3.14, 'up': 3.14}]

# 버튼: 조인트 추가/삭제
col_btns = st.columns([1, 1, 5])
with col_btns[0]:
    if st.button("➕ Add Joint"):
        st.session_state.joints.append({'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'r': 0.0, 'p': 0.0, 'yaw': 0.0, 'low': -3.14, 'up': 3.14})
with col_btns[1]:
    if st.button("🗑️ Reset"):
        st.session_state.joints = [{'axis': 'Roll', 'x': 0.0, 'y': 0.0, 'z': 0.1, 'r': 0.0, 'p': 0.0, 'yaw': 0.0, 'low': -3.14, 'up': 3.14}]
        st.rerun()

# 입력 화면 구성
joint_data_list = []
for i, joint in enumerate(st.session_state.joints):
    with st.expander(f"Joint {i}", expanded=True):
        c1, c2, c3 = st.columns(3)
        with c1:
            axis = st.selectbox(f"Axis##{i}", ["Roll", "Pitch", "Yaw"], index=["Roll", "Pitch", "Yaw"].index(joint['axis']))
        with c2:
            low = st.number_input(f"Lower##{i}", value=joint['low'])
        with c3:
            up = st.number_input(f"Upper##{i}", value=joint['up'])
        
        c4, c5, c6 = st.columns(3)
        with c4: x = st.number_input(f"x##{i}", value=joint['x'])
        with c5: y = st.number_input(f"y##{i}", value=joint['y'])
        with c6: z = st.number_input(f"z##{i}", value=joint['z'])
        
        joint_data_list.append({'axis': axis, 'x': x, 'y': y, 'z': z, 'r': 0, 'p': 0, 'yaw': 0, 'low': low, 'up': up})

# URDF 생성 로직
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

st.divider()
final_urdf = generate_urdf(joint_data_list)
st.subheader("Generated URDF Preview")
st.code(final_urdf, language='xml')

st.download_button(label="💾 Download URDF", data=final_urdf, file_name="robot.urdf")