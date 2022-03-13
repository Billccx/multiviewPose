import typing as tp
import numpy as np
import matplotlib.pyplot as plt


def get_chain_dots(
        dots: np.ndarray,   # shape == (n_dots, 3)
        chain_dots_indexes: tp.List[int], # length == n_dots_in_chain
                                          # in continius order, i.e. 
                                          # left_hand_ix >>> chest_ix >>> right_hand_ix
        ) -> np.ndarray:    # chain of dots
    return dots[chain_dots_indexes]


def get_chains(
        dots: np.ndarray,   # shape == (n_dots, 3)
        spine_chain_ixs: tp.List[int], # pelvis >>> chest >>> head
        hands_chain_ixs: tp.List[int], # left_hand >>> chest >>> right_hand
        legs_chain_ixs: tp.List[int]   # left_leg >>> pelvis >>> right_leg
        ):
    return (get_chain_dots(dots, spine_chain_ixs),
            get_chain_dots(dots, hands_chain_ixs),
            get_chain_dots(dots, legs_chain_ixs))


def subplot_nodes(dots, ax):
    return ax.scatter3D(*dots.T, c=dots[:, -1])


def subplot_bones(chains, ax):
    return [ax.plot(*chain.T) for chain in chains]


def plot_skeletons(skeletons, chains_ixs):
    fig = plt.figure()
    for i, dots in enumerate(skeletons, start=1):
        chains = get_chains(dots, *chains_ixs)
        ax = fig.add_subplot(2, 5, i, projection='3d')
        subplot_nodes(dots, ax)
        subplot_bones(chains, ax)
    plt.show()


def test():
    skeletons = np.random.standard_normal(size=(10, 11, 3))
    chains_ixs = ([0, 1, 2, 3, 4],
                  [5, 2, 6],
                  [7, 8, 5, 9, 10])
    plot_skeletons(skeletons, chains_ixs)


if __name__ == '__main__':
    test()