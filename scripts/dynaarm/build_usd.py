# © 2025 ETH Zurich, Robotic Systems Lab
# Licensed under the Apache License 2.0

"""Build the DynaArm USD from the official Duatic description.

Clones Duatic/duatic_dynaarm at a pinned commit, renders the standalone xacro to URDF, resolves the
``package://`` mesh paths and converts the URDF to USD with Isaac Lab's URDF converter.
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Build the DynaArm USD from the Duatic description.")
parser.add_argument("--commit", type=str, default="718b7b6", help="Commit of Duatic/duatic_dynaarm to use.")
parser.add_argument("--version", type=str, default="baracuda12", help="DynaArm version (baracuda12 or corydoras12).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import shutil
import subprocess
import tempfile
from pathlib import Path

import xacro
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

from pace_sim2real.utils import project_root

DUATIC_REPO = "https://github.com/Duatic/duatic_dynaarm.git"
PACKAGE = "duatic_dynaarm_description"


def main():
    asset_dir = project_root() / "source" / "pace_sim2real" / "pace_sim2real" / "assets" / "dynaarm"
    repo_dir = asset_dir / "duatic_dynaarm"

    # fetch the description at the pinned commit
    if not repo_dir.exists():
        subprocess.run(["git", "clone", "--quiet", DUATIC_REPO, str(repo_dir)], check=True)
    subprocess.run(["git", "-C", str(repo_dir), "fetch", "--quiet", "origin"], check=True)
    subprocess.run(["git", "-C", str(repo_dir), "checkout", "--quiet", args_cli.commit], check=True)
    subprocess.run(["git", "-C", str(repo_dir), "lfs", "pull"], check=True)
    description_dir = repo_dir / PACKAGE

    # the meshes are stored with git-lfs, pointer files mean they were not downloaded
    for mesh in (description_dir / "meshes" / args_cli.version).iterdir():
        if mesh.read_bytes().startswith(b"version https://git-lfs"):
            raise RuntimeError(f"Mesh {mesh} is a git-lfs pointer, the meshes were not downloaded.")

    # render the xacro; $(find ...) needs ROS, so it is substituted in a temporary copy
    with tempfile.TemporaryDirectory() as tmp:
        render_dir = Path(tmp) / PACKAGE
        shutil.copytree(description_dir / "urdf", render_dir / "urdf")
        for xacro_file in render_dir.rglob("*.xacro"):
            xacro_file.write_text(xacro_file.read_text().replace(f"$(find {PACKAGE})", str(render_dir)))
        doc = xacro.process_file(
            str(render_dir / "urdf" / "dynaarm_standalone.urdf.xacro"), mappings={"version": args_cli.version}
        )
        urdf = doc.toprettyxml(indent="  ")

    urdf = urdf.replace(f"package://{PACKAGE}/", f"{description_dir}/")
    urdf_path = asset_dir / f"dynaarm_{args_cli.version}.urdf"
    urdf_path.write_text(urdf)

    cfg = UrdfConverterCfg(
        asset_path=str(urdf_path),
        usd_dir=str(asset_dir / "usd"),
        usd_file_name=f"dynaarm_{args_cli.version}.usd",
        force_usd_conversion=True,
        make_instanceable=True,
        fix_base=True,
        merge_fixed_joints=True,  # merges the 1 g camera mounts and flange into their parents
        self_collision=False,
        # gains are zero since the PACE actuator (explicit model) computes the joint torques
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            drive_type="force",
            target_type="none",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0),
        ),
    )
    converter = UrdfConverter(cfg)
    print(f"[INFO]: Duatic commit: {args_cli.commit}, version: {args_cli.version}")
    print(f"[INFO]: URDF: {urdf_path}")
    print(f"[INFO]: USD: {converter.usd_path}")


if __name__ == "__main__":
    main()
    simulation_app.close()
