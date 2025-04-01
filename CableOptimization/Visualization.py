def set_motor_curves(m):
    global motor_curves
    motor_curves = m


def visualize(cable, browser=False):
    import numpy as np
    import matplotlib
    import matplotlib.pyplot as plt
    from io import BytesIO

    if browser:
        matplotlib.use("Agg")
        import js
        js.document.getElementById('plot').innerHTML = ""

    num_points = 100

    crossections = np.linspace(cable.interval[0], cable.interval[1] + 0.1, num_points)

    # forward flight
    cable.set_hover(False)
    necessary_thrusts_forward = [cable.necessary_thrust(h) for h in crossections]
    power_usages_forward = [cable.total_power(h) for h in crossections]

    minimum = cable.get_minimum_power()
    if minimum != None:
        min_power = cable.total_power(minimum)

    # hover
    cable.set_hover(True)
    necessary_thrusts = [cable.necessary_thrust(h) for h in crossections]
    power_usages = [cable.total_power(h) for h in crossections]
    available_thrusts = [cable.available_thrust(h) for h in crossections]

    solution = cable.get_solution()

    # Create the plot
    fig, ax1 = plt.subplots(figsize=(7, 5))

    # First y-axis for thrust
    if cable.target_velocity > 0:
        ax1.plot(crossections, necessary_thrusts_forward, label="Necessary Thrust", color="cyan")
    ax1.plot(crossections, necessary_thrusts, label="Necessary Thrust Hover", color="blue")
    ax1.plot(crossections, np.array(necessary_thrusts) * 2, label="Target Thrust Hover", color="red")
    ax1.plot(crossections, available_thrusts, linestyle="dotted", label="Available Thrust", color="blue")
    if solution != None:
        plt.axvline(x=solution, color='black', linestyle="dotted")
    ax1.set_xlabel("Cross Section (mm$^2$)")
    ax1.set_ylabel("Thrust (g per motor)", color="blue")
    ax1.tick_params(axis='y', labelcolor="blue")
    ax1.grid(True)

    # Second y-axis for power
    ax2 = ax1.twinx()
    if cable.target_velocity > 0:
        ax2.plot(crossections, power_usages_forward, label="Power Total", color="lime")
    ax2.plot(crossections, power_usages, label="Power Total Hover", color="green")
    if minimum != None:
        plt.plot(minimum, min_power, "o", color="lime", markersize=7)
    ax2.set_ylabel("Power (W)", color="green")
    ax2.tick_params(axis='y', labelcolor="green")

    ax1.legend(loc="upper left")
    ax2.legend(loc="upper left", bbox_to_anchor=(0.425, 1))
    plt.tight_layout()

    if browser:
        buf = BytesIO()
        fig.savefig(buf, format='svg')
        buf.seek(0)
        svg_data = buf.getvalue().decode('utf-8')
        js.document.getElementById('plot').innerHTML = svg_data
        plt.close()
    else:
        plt.show()


def visualize_all(cable, motor_curves, browser=False):
    import numpy as np
    import matplotlib
    import matplotlib.pyplot as plt
    from io import BytesIO

    if browser:
        matplotlib.use("Agg")
        import js
        js.document.getElementById('motorCurves').innerHTML = ""

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 5), gridspec_kw={'width_ratios': [10, 1]})
    ax1.set_ylabel('Efficiency in g/W')
    ax1.set_xlabel('Thrust in g')
    ax2.set_ylabel("Power (W)")
    ax2.yaxis.tick_right()
    ax2.yaxis.set_label_position("right")
    ax2.set_xticks([])

    for key, value in motor_curves.items():
        if len(value) > 1:
            name = value["name"]
            cable.set_motor_curve_dict(value)

            thrusts = cable.motor_curve[0]
            efficiencies = cable.motor_curve[1]

            p = ax1.plot(thrusts, efficiencies, label=name, linewidth=2)

            # forward flight
            cable.set_hover(False)
            minimum = cable.get_minimum_power()
            color = p[0].get_color()

            # hover
            cable.set_hover(True)  # plot efficiency at hover
            solution = cable.get_solution()

            if solution != None:
                necessary_thrust = cable.necessary_thrust(solution)
                ax1.plot(necessary_thrust * 2, np.interp(necessary_thrust * 2, thrusts, efficiencies), "o",
                         color=color,
                         markerfacecolor="none", markersize=7)

                if minimum == None or solution > minimum:
                    ax1.plot(necessary_thrust, np.interp(necessary_thrust, thrusts, efficiencies), "o", color=color,
                             markersize=7)
                    cable.set_hover(False)  # plot power at forward flight
                    ax2.plot(0, cable.necessary_power(solution), "o", color=color, markersize=7)
                    cable.set_hover(True)
                else:
                    necessary_thrust = cable.necessary_thrust(minimum)
                    ax1.plot(necessary_thrust, np.interp(necessary_thrust, thrusts, efficiencies), "P", color=color,
                             markersize=7)
                    cable.set_hover(False)  # plot power at forward flight
                    ax2.plot(0, cable.necessary_power(minimum), "P", color=color, markersize=7)
                    cable.set_hover(True)

    ax1.grid(True)
    ax1.legend()
    plt.tight_layout()

    if browser:
        buf = BytesIO()
        fig.savefig(buf, format='svg')
        buf.seek(0)
        svg_data = buf.getvalue().decode('utf-8')
        js.document.getElementById('motorCurves').innerHTML = svg_data
        plt.close()
    else:
        plt.show()
