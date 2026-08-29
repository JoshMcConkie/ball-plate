import mujoco
import mediapy as media

model = mujoco.MjModel.from_xml_string(
"""
<mujoco>
    <worldbody>
    <geom name="floor" size="0.2 0.2 0.01" type="plane"/>
    <body name="build" pos="0 0 0.01">
        <geom name="stem" fromto="0 0 0 0 0 .12" size="0.03" type="cylinder"/>
        <body name="plate" pos="0 0 .12">
            <joint/>
            <geom size="0.22 0.22 .005"  type="box"/>
        </body>
    </body>
    <geom name="ball" pos="0 0 0.20" size="0.03" type="sphere"/>
    </worldbody>
</mujoco>
""")
data = mujoco.MjData(model)

mujoco.mj_forward(model, data)

with mujoco.Renderer(model) as renderer:
    renderer.update_scene(data)
    media.show_image(renderer.render())