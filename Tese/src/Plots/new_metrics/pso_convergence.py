import matplotlib.pyplot as plt
import numpy as np


def plot_best_objective_over_generations(generations, gbest, save_path=None, show=False):
    """Plot the PSO best objective (J') against generation number.

    Parameters
    ----------
    generations : array-like  Generation indices.
    gbest       : array-like  Best J' found up to each generation.
    """
    generations = np.asarray(generations)
    gbest = np.asarray(gbest, dtype=float)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.semilogy(generations, gbest, linewidth=2.0, marker='o', markersize=3,
                label="Best J'")
    ax.set_title("Best J' over Generations")
    ax.set_xlabel("Generation")
    ax.set_ylabel("Best J'  (log scale)")
    ax.grid(True, which='both', alpha=0.3)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
