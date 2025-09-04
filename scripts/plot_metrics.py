import json
import os
import matplotlib.pyplot as plt

single_size = (8, 4)
double_size = (8, 4)
multi_size = (8, 4)


def plot_multiple_metrics(values_list, timesteps, labels, duration: float = None,
                          save_dir="", save=True, show=True, ylabel="Metric"):
    """plots only the first 'duration' seconds if specified else all"""
    if duration is not None and timesteps:
        filtered = [
            [(t, v) for t, v in zip(timesteps, values) if t <= duration]
            for values in values_list
        ]
        values_list = []
        for data in filtered:
            if data:
                ts, vs = zip(*data)
                timesteps = ts  # assumes all share the same filtered timesteps
                values_list.append(vs)

    plt.figure(figsize=multi_size)
    for values, label in zip(values_list, labels):
        plt.plot(timesteps, values, label=label)

    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    if duration is not None:
        plt.title(f"{' & '.join(labels)} ({int(duration)}s)")
    else:
        plt.title(f"{' & '.join(labels)} vs Time")

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # fix y-limits if all metrics are rates
    if all("rate" in lbl.lower() for lbl in labels):
        plt.ylim(-3, 103)

    if save:
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        fname = "_and_".join(lbl.lower().replace(" ", "_") for lbl in labels) + ".pdf"
        plt.savefig(os.path.join(save_dir, fname))
        print("Plots saved in", save_dir)

    if show:
        plt.show()


def plot_metrics(values: list, timesteps: list, metric_name: str, duration: float = None, save_dir="", save=True, show=True):
    """plots only the first 'duration' seconds if specified else all"""
    if duration is not None and timesteps:
        # Filter data to only include the first 'duration' seconds
        filtered_data = [(t, v) for t, v in zip(timesteps, values) if t <= duration]
        if filtered_data:
            timesteps, values = zip(*filtered_data)
        
    timesteps_filtered, values_filtered = timesteps, values
    
    plt.figure(figsize=single_size)
    plt.plot(timesteps_filtered, values_filtered, label=f'{metric_name} over time')
    
    plt.xlabel('Time (s)')
    plt.ylabel(metric_name)
    
    # Add duration info to title if filtering is applied
    if duration is not None:
        plt.title(f'{metric_name} vs Time ({int(duration)}s)')
    else:
        plt.title(f'{metric_name} vs Time')
    
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
    # if metric is a "rate", fix y-axis between 0 and 100 (with margin)
    if "rate" in metric_name.lower():
        plt.ylim(-3, 103)

    if save:
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        plt.savefig(os.path.join(save_dir, metric_name.lower()+'.pdf'))
        print("Plots saved in", save_dir)
    if show:
        plt.show()


def plot_two_rates(values1: list, values2: list, timesteps: list, 
                   label1: str, label2: str, duration: float = None, 
                   save_dir="", save=True, show=True):
    """plot 2 "rate" metrics over time in the same figure"""
    if duration is not None and timesteps:
        filtered_data1 = [(t, v) for t, v in zip(timesteps, values1) if t <= duration]
        filtered_data2 = [(t, v) for t, v in zip(timesteps, values2) if t <= duration]
        if filtered_data1 and filtered_data2:
            timesteps, values1 = zip(*filtered_data1)
            _, values2 = zip(*filtered_data2)

    plt.figure(figsize=double_size) 
    plt.plot(timesteps, values1, label=label1)
    plt.plot(timesteps, values2, label=label2)

    plt.xlabel('Time (s)')
    plt.ylabel('Rate (%)')
    if duration is not None:
        plt.title(f'{label1} & {label2} vs Time ({int(duration)}s)')
    else:
        plt.title(f'{label1} & {label2} vs Time')

    plt.grid(True)
    plt.legend()
    plt.ylim(-3, 103)
    plt.tight_layout()

    if save:
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        fname = f"{label1.lower().replace(' ', '_')}_and_{label2.lower().replace(' ', '_')}.pdf"
        plt.savefig(os.path.join(save_dir, fname))
        print("Plots saved in", save_dir)

    if show:
        plt.show()


def load_metrics(fpath="metrics.json"):
    with open(fpath, "r") as f:
        data = json.load(f)
    return data



if __name__ == "__main__":
    load_path = "TODO"
    save_dir = "TODO"
    
    metrics = load_metrics(load_path)
    
    #plot_metrics(metrics["coverage_rates"], metrics["timesteps"], "Coverage Rate", duration=120.0, save_dir=save_dir, save=True, show=True)
    #plot_metrics(metrics["entering_rates"], metrics["timesteps"], "Entering Rate", duration=120.0, save_dir=save_dir, save=True, show=True)
    #plot_metrics(metrics["distribution_uniformities"], metrics["timesteps"], "Distribution Uniformity", duration=120.0, save_dir=save_dir, save=True, show=True)
    #plot_metrics(metrics["velocity_polarizations"], metrics["timesteps"], "Velocity Polarization", duration=120.0, save_dir=save_dir, save=True, show=True)

    # plot both rates together
    plot_multiple_metrics(
        [metrics["entering_rates"], metrics["coverage_rates"]],
        metrics["timesteps"],
        ["Coverage Rate", "Entering Rate"],
        duration=120.0, save_dir=save_dir, save=True, show=True, ylabel="Rate (%)"
    )

    # plot both non-rates together
    plot_multiple_metrics(
        [metrics["distribution_uniformities"], metrics["velocity_polarizations"]],
        metrics["timesteps"],
        ["Distribution Uniformity", "Velocity Polarization"],
        duration=120.0, save_dir=save_dir, save=True, show=True, ylabel="Value"
    )