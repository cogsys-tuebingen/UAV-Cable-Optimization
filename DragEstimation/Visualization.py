def visualize_input(drag, browser=False):
    import matplotlib
    import matplotlib.pyplot as plt
    from io import BytesIO

    if browser:
        matplotlib.use("Agg")
        import js
        js.document.getElementById('input_data').innerHTML = ""
    else:
        matplotlib.use("Qt5Agg")

    fig = plt.figure(figsize=(6, 3))

    plt.plot(drag.df['time'] / 1000000, drag.df['AX_e'], label='expected acceleration', color='b', linewidth=0.5)
    plt.plot(drag.df['time'] / 1000000, drag.df['AX_o'], label='observed acceleration', color='dodgerblue',
             linewidth=0.5)
    plt.plot(drag.df['time'] / 1000000, drag.df['DX'], label=f'drag ($\\sigma=${drag.noise_stdev:.2f})', color='orange',
             linewidth=0.5)
    plt.plot(drag.df['time'] / 1000000, drag.smoothed_drags, color='r', linewidth=0.5)

    plt.xlabel('Time (seconds)')
    plt.ylabel('Acceleration (m/s$^2$)')
    plt.legend(loc="lower left")
    plt.grid(True)
    plt.tight_layout()

    if browser:
        buf = BytesIO()
        fig.savefig(buf, format='svg', bbox_inches='tight')
        buf.seek(0)
        svg_data = buf.getvalue().decode('utf-8')
        js.document.getElementById('input_data').innerHTML = svg_data
        plt.close()
    else:
        plt.show()


def visualize_drag(drag, browser=False):
    import matplotlib
    import matplotlib.pyplot as plt
    import numpy as np
    from io import BytesIO

    if browser:
        matplotlib.use("Agg")
        import js
        js.document.getElementById('result_2D').innerHTML = ""
        js.document.getElementById('result_3D').innerHTML = ""
    else:
        matplotlib.use("Qt5Agg")

    # 2D plot-----------------------------------------------------------------------------------------------------------
    # remove wind offset --> ensure symmetry
    velocities = drag.velocities - drag.popt[-1]
    pitches = drag.pitches
    popt = drag.popt
    popt[-1] = 0
    drag_estimates = drag.drag_model((velocities, pitches), *popt) / drag.drone_mass

    velocity_range = np.linspace(velocities.min(), velocities.max(), 100)
    pitch_range = np.linspace(pitches.min(), pitches.max(), 50)

    equilibrium_pitches = [drag.equilibrium_angle(x, drag.drone_mass, *popt) for x in velocity_range]
    equilibrium_drags = drag.gravity * np.tan(np.radians(equilibrium_pitches))

    residuals = drag.drags - drag_estimates
    mae = np.mean(np.abs(residuals))

    fig = plt.figure(figsize=(6, 4.5))
    plt.scatter(velocities, residuals, color='g', alpha=0.1,
                label=f'residuals (MAE {mae:.2f})')
    plt.scatter(velocities, drag.drags, color='b', alpha=0.05, label='measured drag')
    plt.scatter(velocities, drag_estimates, color='r', alpha=0.05, label='estimated drag')
    plt.plot(velocity_range, equilibrium_drags, color='y', label='equilibrium')
    legend = plt.legend(loc="upper left")

    for handle in legend.legend_handles:
        handle.set_alpha(1.0)

    plt.xlabel('Velocity (m/s)')
    plt.ylabel('Drag (m/s$^2$)')
    plt.tight_layout()
    plt.grid(True)

    if browser:
        buf = BytesIO()
        fig.savefig(buf, format='svg', bbox_inches='tight')
        buf.seek(0)
        svg_data = buf.getvalue().decode('utf-8')
        js.document.getElementById('result_2D').innerHTML = svg_data
        plt.close()
    else:
        plt.show()

    # 3d plot-----------------------------------------------------------------------------------------------------------
    if not browser:
        matplotlib.use('Qt5Agg')

    velocity_grid, pitch_grid = np.meshgrid(velocity_range, pitch_range)
    drag_grid = drag.drag_model((velocity_grid, pitch_grid), *popt) / drag.drone_mass

    fig = plt.figure(figsize=(6, 4.5))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)

    ax.plot_surface(pitch_grid, velocity_grid, drag_grid, color='r', edgecolor='none', alpha=0.3,
                    zorder=0, label='estimated drag')
    # ax.scatter(pitches, velocities, drags, color='r', alpha=0.1)
    r = ax.scatter(pitches, velocities, residuals, c=residuals, alpha=0.2, cmap='viridis',
                   label=f'residuals (MAE {mae:.2f})')
    ax.plot(equilibrium_pitches, velocity_range, equilibrium_drags, color='r', alpha=0.4, label='equilibrium')

    ax.set_xlabel('Pitch (degrees)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_zlabel('Drag (m/s$^2$)')
    ax.view_init(elev=30, azim=-135)
    fig.colorbar(r, shrink=0.5, aspect=10)
    plt.legend(loc="upper left")
    plt.tight_layout()

    if browser:
        buf = BytesIO()
        fig.savefig(buf, format='svg', bbox_inches='tight')
        buf.seek(0)
        svg_data = buf.getvalue().decode('utf-8')
        js.document.getElementById('result_3D').innerHTML = svg_data
        plt.close()
    else:
        plt.show()
