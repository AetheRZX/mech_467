from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Tuple

import control
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use("Agg")

# --- System parameters (SI units unless noted) ---
KA = 0.887  # A/V
KT = 0.72  # Nm/A
JE = 7e-4  # kg*m^2
BE = 0.00612  # N*m*s/rad
KE = 20 / (2 * math.pi)  # mm/rad
T_S = 2e-4  # s

MU_C = 0.3  # Nm Coulomb friction


def build_plant() -> control.TransferFunction:
    """Continuous-time open-loop transfer function from Vin to Xa."""
    num = [KA * KT * KE]
    den = [JE, BE, 0.0]
    return control.TransferFunction(num, den)


def build_state_space():
    """Continuous-time state space (states: omega, position)."""
    a = np.array([[-BE / JE, 0.0], [KE, 0.0]])
    b = np.array([[KA * KT / JE], [0.0]])
    c = np.array([[0.0, 1.0]])
    d = np.array([[0.0]])
    return control.ss(a, b, c, d)


def convert_to_discrete(tf: control.TransferFunction, ts: float) -> control.TransferFunction:
    return control.c2d(tf, ts, method="zoh")


def discrete_state_space(ts: float) -> control.StateSpace:
    sys_c = build_state_space()
    return control.c2d(sys_c, ts)


def compare_step_responses(gz: control.TransferFunction, ssd: control.StateSpace, ts: float, path: Path):
    t = np.arange(0.0, 0.05, ts)
    tout, y_tf = control.forced_response(gz, T=t, U=np.ones_like(t))
    tout_ss, y_ss = control.forced_response(ssd, T=t, U=np.ones_like(t))

    plt.figure(figsize=(6, 4))
    plt.plot(tout, y_tf.squeeze(), label="Transfer Function")
    plt.plot(tout_ss, y_ss.squeeze(), "--", label="State-Space")
    plt.xlabel("Time [s]")
    plt.ylabel("Position [mm]")
    plt.title("Discrete Step Response Comparison")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path / "step_compare.png", dpi=180)
    plt.close()


def compare_bode_models(gz: control.TransferFunction, ssd: control.StateSpace, path: Path):
    omega = np.logspace(0, 2.2, 450)
    mag_tf, phase_tf, _ = control.freqresp(gz, omega)
    mag_ss, phase_ss, _ = control.freqresp(ssd, omega)

    fig, ax = plt.subplots(2, 1, figsize=(6.5, 5.5), sharex=True)
    ax[0].semilogx(omega, 20 * np.log10(mag_tf.squeeze()), label="Transfer Function")
    ax[0].semilogx(omega, 20 * np.log10(mag_ss.squeeze()), "--", label="State-Space")
    ax[0].set_ylabel("Magnitude [dB]")
    ax[0].grid(True, which="both")
    ax[0].legend()

    ax[1].semilogx(omega, np.rad2deg(phase_tf.squeeze()))
    ax[1].semilogx(omega, np.rad2deg(phase_ss.squeeze()), "--")
    ax[1].set_ylabel("Phase [deg]")
    ax[1].set_xlabel("Frequency [rad/s]")
    ax[1].grid(True, which="both")

    fig.suptitle("Discrete Model Frequency Response Comparison")
    fig.tight_layout()
    fig.savefig(path / "bode_model_compare.png", dpi=200)
    plt.close(fig)


def plot_root_locus(
    sys: control.TransferFunction,
    path: Path,
    name: str,
    limits=None,
    equal_axes=True,
    unit_circle=False,
    damping_lines: list[float] | None = None,
):
    plt.figure(figsize=(6.3, 6.0))
    rlist, _ = control.root_locus(sys, plot=False)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for idx in range(rlist.shape[1]):
        branch = rlist[:, idx]
        color = colors[idx % len(colors)]
        plt.plot(branch.real, branch.imag, color=color, linewidth=2)
        plt.scatter(
            branch[0].real,
            branch[0].imag,
            color=color,
            marker="x",
            s=70,
            linewidths=2,
        )
        plt.scatter(
            branch[-1].real,
            branch[-1].imag,
            color=color,
            marker="o",
            facecolors="none",
            s=60,
            linewidths=2,
        )
    if unit_circle:
        theta = np.linspace(0, 2 * np.pi, 400)
        plt.plot(np.cos(theta), np.sin(theta), "k--", linewidth=1)
    ax = plt.gca()
    if limits:
        xlim, ylim = limits
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
    if equal_axes:
        ax.set_aspect("equal", adjustable="box")
    else:
        ax.set_aspect("auto")
    plt.grid(True, which="both")
    plt.title(f"{name} Root Locus")
    if damping_lines:
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        radius = max(abs(xlim[0]), abs(ylim[0]), abs(ylim[1])) * 1.2
        x_vals = np.linspace(0.0, -radius, 300)
        for zeta in damping_lines:
            if zeta >= 1.0:
                continue
            angle = math.acos(zeta)
            slope = math.tan(angle)
            y_vals = slope * (-x_vals)
            ax.plot(x_vals, y_vals, linestyle=":", color="gray", linewidth=0.8)
            ax.plot(x_vals, -y_vals, linestyle=":", color="gray", linewidth=0.8)
            ax.text(
                x_vals[-1],
                y_vals[-1],
                f"{zeta:.2f}",
                fontsize=8,
                color="gray",
                ha="right",
            )
    plt.tight_layout()
    plt.savefig(path / f"root_locus_{name.lower()}.png", dpi=180)
    plt.close()


def bode_data(
    sys,
    omega: np.ndarray,
    unwrap_phase: bool = False,
    wrap_phase: bool = True,
) -> Tuple[np.ndarray, np.ndarray]:
    mag, phase, _ = control.freqresp(sys, omega)
    phase = phase.squeeze()
    if unwrap_phase:
        phase = np.unwrap(phase)
    phase_deg = np.rad2deg(phase)
    if wrap_phase:
        phase_deg = (phase_deg + 180.0) % 360.0 - 180.0
    return 20 * np.log10(mag.squeeze()), phase_deg


def plot_bode_comparison(gs, gz, path: Path):
    omega = np.logspace(0, 2.2, 800)
    mag_s, ph_s = bode_data(gs, omega, unwrap_phase=True, wrap_phase=False)
    mag_z, ph_z = bode_data(gz, omega, unwrap_phase=True, wrap_phase=False)

    fig, ax = plt.subplots(2, 1, figsize=(6.5, 6), sharex=True)
    ax[0].semilogx(omega, mag_s, label="G(s)", alpha=0.8, linewidth=2)
    ax[0].semilogx(omega, mag_z, label="G(z)", alpha=0.8, linewidth=1.5)
    ax[0].set_ylabel("Magnitude [dB]")
    ax[0].grid(True, which="both")
    ax[0].legend()

    ax[1].semilogx(omega, ph_s, alpha=0.8, linewidth=2)
    ax[1].semilogx(omega, ph_z, alpha=0.8, linewidth=1.5)
    ax[1].set_ylabel("Phase [deg]")
    ax[1].set_xlabel("Frequency [rad/s]")
    ax[1].grid(True, which="both")

    fig.suptitle("Continuous vs Discrete Bode")
    fig.tight_layout()
    fig.savefig(path / "bode_compare.png", dpi=200)
    plt.close(fig)


    plt.close()


def plot_manual_zoh_vs_c2d(gs, path: Path):
    # Manual Discrete TF from Report Eq:
    # num = 5.8048e-5 z + 5.8014e-5
    # den = z^2 - 1.99825296 z + 0.99825296
    
    omega = np.logspace(0, 4, 800)
    
    # "Auto" (Python c2d)
    gz_auto = convert_to_discrete(gs, T_S)
    mag_auto, phase_auto, _ = control.freqresp(gz_auto, omega)
    
    # "Manual"
    # Construct TF directly
    num_man = [5.8048e-5, 5.8014e-5]
    den_man = [1.0, -1.99825296, 0.99825296]
    gz_manual = control.TransferFunction(num_man, den_man, dt=T_S)
    
    mag_manual, phase_manual, _ = control.freqresp(gz_manual, omega)
    
    fig, ax = plt.subplots(2, 1, figsize=(6.5, 6), sharex=True)
    
    # Magnitude
    ax[0].semilogx(omega, 20*np.log10(mag_auto.squeeze()), 'b', linewidth=2, label="c2d command")
    ax[0].semilogx(omega, 20*np.log10(mag_manual.squeeze()), 'r--', linewidth=2, label="Manual G(z)")
    ax[0].set_ylabel("Magnitude [dB]")
    ax[0].grid(True, which="both")
    ax[0].legend()
    
    # Phase
    # Unwrap manually or use bode_data if available, but simple conversion is fine here
    ph_auto_deg = np.rad2deg(phase_auto.squeeze())
    ph_man_deg = np.rad2deg(phase_manual.squeeze())
    
    ax[1].semilogx(omega, ph_auto_deg, 'b', linewidth=2)
    ax[1].semilogx(omega, ph_man_deg, 'r--', linewidth=2)
    ax[1].set_ylabel("Phase [deg]")
    ax[1].set_xlabel("Frequency [rad/s]")
    ax[1].grid(True, which="both")
    
    fig.suptitle("Manual G(z) vs c2d")
    fig.tight_layout()
    fig.savefig(path / "bode_manual_vs_c2d.png", dpi=200)
    plt.close(fig)


def plot_sampling_bodes(gs, samples: Iterable[float], path: Path):
    # Added "0.02, 0.002, 0.0002" specifically per user request
    omega = np.logspace(0, 5, 800)
    plt.figure(figsize=(7, 5))
    
    colors = ['r', 'g', 'b']
    for idx, ts in enumerate(samples):
        gz = convert_to_discrete(gs, ts)
        mag, _ = bode_data(gz, omega)
        
        # Calculate Gain Margin for the table values
        gm, pm, wcg, wcp = control.margin(gz)
        gm_db = 20 * np.log10(gm) if gm > 0 else 0
        
        plt.semilogx(omega, mag, label=f"T={ts}s (GM={gm_db:.2f} dB)", color=colors[idx % len(colors)])
        
    plt.xlabel("Frequency [rad/s]")
    plt.ylabel("Magnitude [dB]")
    plt.title("Discrete Bode Magnitude vs Sampling Time")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path / "bode_sampling.png", dpi=200)
    plt.close()


def plot_pcontroller_bode(gz, kp: float, path: Path):
    """Bode of the discrete open loop with the selected Kp."""
    omega = np.logspace(0, 3.5, 700)
    loop = kp * gz
    mag, phase_deg = bode_data(loop, omega, unwrap_phase=True, wrap_phase=False)

    gm, pm, wcg, wcp = control.margin(loop)
    # gm is linear gain, convert to dB
    # control.margin returns gm, pm, wcg (phase crossover), wcp (gain crossover)
    # NOTE: In python-control < 0.9, the order was different. Assuming modern version.
    # Wcg is frequency where phase is -180 (for GM calculation)
    # Wcp is frequency where mag is 0 dB (for PM calculation)

    fig, ax = plt.subplots(2, 1, figsize=(6.3, 6.5), sharex=True)
    ax[0].semilogx(omega, mag, label=r"$K_p G_\text{ol}(z)$")
    ax[0].axhline(0.0, color="k", linestyle="-", linewidth=0.8)
    
    # Mark GM
    if gm > 0 and not math.isinf(gm):
        gm_db = 20 * np.log10(gm)
        # At wcg, gain is -gm_db. 
        # Actually standard definition: GM is how much we can raise gain.
        # So at wcg, existing magnitude is -GM_dB.
        mag_at_wcg = -gm_db
        ax[0].scatter([wcg], [mag_at_wcg], color='r', marker='o', s=50, zorder=5)
        ax[0].text(wcg, mag_at_wcg + 5, f"GM={gm_db:.1f} dB", color='r', fontsize=9, ha='center')

    # Mark Gain Crossover (0 dB)
    ax[0].scatter([wcp], [0], color='g', marker='o', s=50, zorder=5)

    ax[0].set_ylabel("Magnitude [dB]")
    ax[0].grid(True, which="both")
    ax[0].legend()

    ax[1].semilogx(omega, phase_deg)
    ax[1].axhline(-180, color='k', linestyle='-', linewidth=0.8)
    
    # Mark PM
    # At wcp, phase is -180 + PM
    phase_at_wcp = -180 + pm
    ax[1].scatter([wcp], [phase_at_wcp], color='g', marker='o', s=50, zorder=5)
    ax[1].text(wcp, phase_at_wcp + 10, f"PM={pm:.1f} deg", color='g', fontsize=9, ha='center')
    
    # Mark Phase Crossover (-180)
    if gm > 0:
        ax[1].scatter([wcg], [-180], color='r', marker='o', s=50, zorder=5)

    ax[1].set_ylabel("Phase [deg]")
    ax[1].set_xlabel("Frequency [rad/s]")
    ax[1].grid(True, which="both")

    fig.suptitle(f"Bode of Discrete Loop with $K_p={kp:.2f}$ V/mm")
    fig.tight_layout()
    fig.savefig(path / "pcontroller_bode.png", dpi=200)
    plt.close(fig)


def simulate_p_controller(kp: float, mu_vals, sat_vals, path: Path):
    def friction_torque(omega: float, mu: float) -> float:
        if mu == 0.0:
            return 0.0
        return mu * math.tanh(omega / 1e-4)

    def simulate(mu: float, sat: float, t_final: float = 0.08):
        steps = int(t_final / T_S)
        omega = 0.0
        pos = 0.0
        traj = []
        time = []
        for k in range(steps):
            err = 1.0 - pos
            vin = kp * err
            ia = max(-sat, min(sat, KA * vin))
            tau = KT * ia - friction_torque(omega, mu)
            domega = (tau - BE * omega) / JE
            omega += domega * T_S
            pos += KE * omega * T_S
            traj.append(pos)
            time.append(k * T_S)
        return np.array(time), np.array(traj)

    fig, ax = plt.subplots(2, 1, figsize=(6.2, 6), sharex=True)
    friction_curves = []
    for mu in mu_vals:
        t, y = simulate(mu, sat=3.0)
        friction_curves.append((mu, t, y))
        ax[0].plot(t, y, label=f"mu={mu:.1f} Nm")
    ax[0].set_title("Effect of Coulomb Friction (sat=+/-3A)")
    ax[0].set_ylabel("Position [mm]")
    ax[0].grid(True, which="both")
    ax[0].legend()

    saturation_curves = []
    for sat in sat_vals:
        t, y = simulate(mu=0.3, sat=sat)
        saturation_curves.append((sat, t, y))
        ax[1].plot(t, y, label=f"sat=+/-{sat:.1f} A")
    ax[1].set_title("Effect of Current Saturation (mu=0.3 Nm)")
    ax[1].set_xlabel("Time [s]")
    ax[1].set_ylabel("Position [mm]")
    ax[1].grid(True, which="both")
    ax[1].legend()

    fig.tight_layout()
    fig.savefig(path / "pcontroller_nonlinear_effects.png", dpi=200)
    plt.close(fig)

    # Dedicated plots for report
    plt.figure(figsize=(6.2, 4.0))
    for mu, t, y in friction_curves:
        plt.plot(t * 1000.0, y, label=f"mu_k={mu:.1f} Nm")
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.title("Step Response vs Coulomb Friction (sat = +/-3 A)")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path / "pcontroller_friction_steps.png", dpi=200)
    plt.close()

    plt.figure(figsize=(6.2, 4.0))
    for sat, t, y in saturation_curves:
        plt.plot(t * 1000.0, y, label=f"sat = +/-{sat:.1f} A")
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.title("Step Response vs Saturation Limit (mu_k = 0.3 Nm)")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path / "pcontroller_saturation_steps.png", dpi=200)
    plt.close()


def design_lead_lag(gs):
    omega_c = 377.0
    mag, phase, _ = control.freqresp(gs, np.array([omega_c]))
    mag = mag.squeeze()
    phase = math.degrees(phase.squeeze())
    pm_current = 180.0 + phase
    phi_needed = 60.0 - pm_current
    phi_rad = math.radians(phi_needed)
    alpha = (1 + math.sin(phi_rad)) / (1 - math.sin(phi_rad))
    tau = 1 / (omega_c * math.sqrt(alpha))
    num = [alpha * tau, 1.0]
    den = [tau, 1.0]
    lead = control.TransferFunction(num, den)
    mag_lead, _, _ = control.freqresp(lead, np.array([omega_c]))
    k = 1.0 / (mag * mag_lead.squeeze())
    return lead * k, {"phase_deg": phase, "pm_current": pm_current, "phi_needed": phi_needed, "alpha": alpha, "tau": tau, "k": k}


def simulate_compensators(gs, lead, path: Path):
    omega = np.logspace(0, 4, 800)
    mag, phase = bode_data(lead, omega, unwrap_phase=True, wrap_phase=False)
    plt.figure(figsize=(6.2, 4.5))
    plt.semilogx(omega, mag)
    plt.title("Lead Compensator Bode")
    plt.xlabel("Frequency [rad/s]")
    plt.ylabel("Magnitude [dB]")
    plt.grid(True, which="both")
    plt.tight_layout()
    plt.savefig(path / "lead_bode.png", dpi=200)
    plt.close()

    loop = control.series(lead, gs)
    omega = np.logspace(0, 4, 800)
    mag_loop, phase_loop = bode_data(loop, omega, unwrap_phase=True, wrap_phase=False)
    # Calculate margins for Loop with Lead
    gm, pm, wcg, wcp = control.margin(loop)
    
    fig, ax = plt.subplots(2, 1, figsize=(6.2, 6.5), sharex=True)
    ax[0].semilogx(omega, mag_loop)
    ax[0].axhline(0, color="k", linestyle="-", linewidth=0.8)
    
    if gm > 0 and not math.isinf(gm):
        gm_db = 20 * np.log10(gm)
        mag_at_wcg = -gm_db
        ax[0].scatter([wcg], [mag_at_wcg], color='r', marker='o', s=50, zorder=5)
        ax[0].text(wcg, mag_at_wcg + 5, f"GM={gm_db:.1f} dB", color='r', fontsize=9, ha='center')
    
    ax[0].text(wcp, 0 + 2, "0 dB", color='g', fontsize=8, ha='center')
    ax[0].scatter([wcp], [0], color='g', marker='o', s=50, zorder=5)

    ax[0].set_ylabel("Magnitude [dB]")
    ax[0].grid(True, which="both")
    
    ax[1].semilogx(omega, phase_loop)
    ax[1].axhline(-180, color='k', linestyle="-", linewidth=0.8)
    
    phase_at_wcp = -180 + pm
    ax[1].scatter([wcp], [phase_at_wcp], color='g', marker='o', s=50, zorder=5)
    ax[1].text(wcp, phase_at_wcp + 7, f"PM={pm:.1f} deg", color='g', fontsize=9, ha='center')
    
    if gm > 0:
        ax[1].scatter([wcg], [-180], color='r', marker='o', s=50, zorder=5)

    ax[1].set_ylabel("Phase [deg]")
    ax[1].set_xlabel("Frequency [rad/s]")
    ax[1].grid(True, which="both")
    
    fig.suptitle("Loop Return Ratio with Lead")
    fig.tight_layout()
    fig.savefig(path / "lrr_lead.png", dpi=200)
    plt.close(fig)

    ki = 377.0 / 10.0
    integrator = control.TransferFunction([1.0, ki], [1.0, 0.0])
    lead_lag = lead
    lead_lag_int = control.series(lead_lag, integrator)

    systems = {
        "Gol(s)": gs,
        "LL(s)": lead_lag,
        "LLI(s)": lead_lag_int,
        "Gol*LL": control.series(gs, lead_lag),
        "Gol*LLI": control.series(gs, lead_lag_int),
    }
    
    # Calculate margins for LLI
    loop_lli = control.series(gs, lead_lag_int)
    gm_lli, pm_lli, wcg_lli, wcp_lli = control.margin(loop_lli)

    omega = np.logspace(0, 4, 800)
    fig, ax = plt.subplots(2, 1, figsize=(7.0, 7.0), sharex=True)
    for label, sys in systems.items():
        mag, phase = bode_data(sys, omega, unwrap_phase=True, wrap_phase=False)
        ax[0].semilogx(omega, mag, label=label)
        ax[1].semilogx(omega, phase)
    
    ax[0].set_ylabel("Magnitude [dB]")
    ax[0].grid(True, which="both")
    ax[0].legend(fontsize="small", loc='lower left')
    ax[1].set_ylabel("Phase [deg]")
    ax[1].set_xlabel("Frequency [rad/s]")
    ax[1].grid(True, which="both")
    
    # Mark LLI margins on the summary plot if feasible, or just leave clean "human" style
    # User asked for "points you know". We can add them for LLI trace.
    # LLI trace is "Gol*LLI"
    # if gm_lli > 0 and not math.isinf(gm_lli):
    #    gm_db = 20 * np.log10(gm_lli)
    #    ax[0].plot([wcg_lli], [-gm_db], 'ro') # red dot
    #    ax[1].plot([wcg_lli], [-180], 'ro')
    # ax[0].plot([wcp_lli], [0], 'go') # green dot
    # ax[1].plot([wcp_lli], [-180 + pm_lli], 'go')

    fig.suptitle("Bode Summary")
    fig.tight_layout()
    fig.savefig(path / "bode_summary.png", dpi=200)
    plt.close(fig)

    return lead_lag, lead_lag_int, ki


def simulate_reference_tracking(gs, lead, lead_int, path: Path):
    ki = 377.0 / 10.0
    integrator = control.TransferFunction([1.0, ki], [1.0, 0.0])
    lead_int_full = control.series(lead, integrator)
    vf = MU_C / (KA * KT)

    def build_closed_loop(controller):
        loop = control.series(controller, gs)
        tref = control.feedback(loop, 1)
        tdist = control.feedback(gs, controller)
        return tref, tdist

    tref_lead, tdist_lead = build_closed_loop(lead)
    tref_int, tdist_int = build_closed_loop(lead_int_full)

    t = np.linspace(0, 0.2, 1000)
    _, y_step_lead = control.step_response(tref_lead, T=t)
    _, y_step_int = control.step_response(tref_int, T=t)
    _, y_dist_lead = control.forced_response(tdist_lead, T=t, U=-vf * np.ones_like(t))
    _, y_dist_int = control.forced_response(tdist_int, T=t, U=-vf * np.ones_like(t))

    y_step_total_lead = y_step_lead + y_dist_lead
    y_step_total_int = y_step_int + y_dist_int

    ramp = t
    _, y_ramp_lead = control.forced_response(tref_lead, T=t, U=ramp)
    _, y_ramp_int = control.forced_response(tref_int, T=t, U=ramp)
    y_ramp_total_lead = y_ramp_lead + y_dist_lead
    y_ramp_total_int = y_ramp_int + y_dist_int

    fig, ax = plt.subplots(2, 1, figsize=(6.4, 6), sharex=True)
    ax[0].plot(t, y_step_total_lead, label="Lead only")
    ax[0].plot(t, y_step_total_int, label="Lead + Integral")
    ax[0].set_ylabel("Position [mm]")
    ax[0].set_title("Step Tracking with Friction Disturbance")
    ax[0].grid(True, which="both")
    ax[0].legend()

    ax[1].plot(t, y_ramp_total_lead, label="Lead only")
    ax[1].plot(t, y_ramp_total_int, label="Lead + Integral")
    ax[1].set_ylabel("Position [mm]")
    ax[1].set_xlabel("Time [s]")
    ax[1].set_title("Ramp Tracking with Friction Disturbance")
    ax[1].grid(True, which="both")
    ax[1].legend()

    fig.tight_layout()
    fig.savefig(path / "lead_integral_tracking.png", dpi=200)
    plt.close(fig)


def main():
    fig_dir = Path("figures")
    fig_dir.mkdir(exist_ok=True)

    gs = build_plant()
    gz = convert_to_discrete(gs, T_S)
    ssd = discrete_state_space(T_S)

    compare_step_responses(gz, ssd, T_S, fig_dir)
    compare_bode_models(gz, ssd, fig_dir)
    continuous_limits = ((-15.0, 2.0), (-12.0, 12.0))
    discrete_limits = ((-3.5, 1.5), (-2.2, 2.2))
    plot_root_locus(
        gs,
        fig_dir,
        "Continuous",
        limits=continuous_limits,
        equal_axes=False,
        damping_lines=[0.2, 0.4, 0.6, 0.8, 0.95],
    )
    plot_root_locus(
        gz,
        fig_dir,
        "Discrete",
        limits=discrete_limits,
        equal_axes=True,
        unit_circle=True,
    )
    plot_bode_comparison(gs, gz, fig_dir)
    plot_manual_zoh_vs_c2d(gs, fig_dir)
    plot_sampling_bodes(gs, [0.02, 0.002, 0.0002], fig_dir)

    mag, _, _ = control.freqresp(gz, np.array([60.0]))
    kp = 1.0 / mag.squeeze()
    plot_pcontroller_bode(gz, kp, fig_dir)
    simulate_p_controller(kp, mu_vals=[0.5, 0.3, 0.1, 0.0], sat_vals=[0.5, 1.0, 3.0], path=fig_dir)

    lead, lead_meta = design_lead_lag(gs)
    lead_lag, lead_lag_int, _ = simulate_compensators(gs, lead, fig_dir)
    simulate_reference_tracking(gs, lead_lag, lead_lag_int, fig_dir)

    print("Lead design meta:", lead_meta)
    print("Kp (unity gain at 60 rad/s):", kp)


if __name__ == "__main__":
    main()
