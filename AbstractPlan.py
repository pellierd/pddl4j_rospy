from Plan import Plan
from Action import Action


class AbstractPlan (Plan):
    """
    This abstract class implements the main methods of a search.
    """

    def __init__(self, plan=None):
        super
        if isinstance(plan, Plan):
            self._size = plan.size()
            self._makespan = plan.makespan()
            self._cost = plan.cost()
            self._time_specifiers = plan.time_specifiers()
            self._actions = plan.actions()

    @staticmethod
    def is_list_action(liste_actions):
        """
        This function return true if the list contains only Actions
        :param liste_actions: the list that need to be checked
        :return: <code>true</code> if the list contains only actions; <code>false</code> otherwise.
        """
        bool_is_liste_action = True
        for elem in liste_actions:
            if not isinstance(elem,Action):
                bool_is_liste_action = False
        return bool_is_liste_action

    def size(self):
        """
        Returns the size of the search. The size of the search is its number of actions.
        :return: the size of the search.
        """
        taille = 0
        for elem in self.time_specifiers():
            taille += len(self.get_actions_set(elem))
        return taille

    def cost(self):
        """
        Returns the cost of the search. The cost of a search is the sum of the cost of its actions.
        :return: the cost of the search.
        """
        cout = 0
        for elem in self.time_specifiers():
            for action in self.get_actions_set(elem):
                cout += action.cost

        return cout

    def is_empty(self):
        """
        Returns if the search is empty.s
        :return: <code>true</code> if the search is empty; <code>false</code> otherwise.
        see Plan#isEmpty()
        """
        if self.size() == 0:
            return True

