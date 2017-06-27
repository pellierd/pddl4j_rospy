#!/usr/bin/env python
from AbstractPlan import AbstractPlan
from Action import Action
from copy import copy


class SequentialPlan (AbstractPlan):
    """
    This class allow us to use Sequential plan
    """
    def __init__(self, actions=None):
        """
        Constructor of the class
        :param actions: a lsit of actions
        """
        super
        if isinstance(actions, list) and SequentialPlan.is_list_action(actions):
            self._actions = list()

            for elem in actions:
                self._actions.append(elem)
        else:
            self._actions = list()

    @staticmethod
    def is_list_action(liste_actions):
        """
        return if the liste contains only actions
        :param liste_actions:
        :return: <code>True</code> if the list contains only actions
        """
        bool_is_liste_action = True
        for elem in liste_actions:
            if not isinstance(elem,Action):
                bool_is_liste_action = False
        return bool_is_liste_action

    def makespan(self):
        """
        Returns the makespan of the plan. The makespan of a sequential plan is its size.
        :return: the makespan of the plan.
        see Plan#size()
        """
        return self.size()

    def actions(self):
        """
        Returns the list of actions contained in the plan ordered depending on their time specifier

        :return: the ordered set of actions of the plan.
        see Plan#actions()
        """
        return self._actions

    def time_specifiers(self):
        """
        Returns the set of time specifiers used in this plan.
        :return: the set of time specifiers used in this plan.
        see Plan#timeSpecifiers()
        """
        ts = list()
        for i in range(0, len(self._actions)):
            ts.append(i)

        return ts

    def get_actions_set(self, time):
        """
        Returns the set of actions at a specified time specifier.
        :param time: the time specifier.
        :return: the set of actions at a specified time specifier or null if no actions are scheduled in the plan at the
        the time specifier.
        see Plan#get_action_set(int)
        """
        if isinstance(time, int):
            if self.is_time_specifiers_out_of_bound(time):
                return None
            ts = list()
            ts.append(self._actions[time])
            return ts
        else:
            raise SyntaxError("Error in function get_actions_set in class Sequential_plan.py,"
                              "the value of the parameter is not a int")

    def add(self, time, action):
        """
        Adds an action at a specified time specifier in the plan.
        :param time: the action to add.
        :param action: the time specifier of the action in the plan.
        :return: <code>true</code> if the action was added; <code>false</code> otherwise.
        see Plan#add(int, Action)
        """
        if isinstance(time, int):
            if self.is_time_specifiers_out_of_bound(time):
                return False
        self._actions[int(time)] = action

    def remove(self, time, action=None):
        """
        :param time:the time specifier of the action in the plan to remove.
        :param action:the action to remove.
        :return: <code>true</code> if the action was added; <code>false</code> otherwise.
        see Plan#remove(int, time, action)
        """
        if isinstance(action, Action) \
                and self.get_actions_set(time)[0].name == action.name and isinstance(time, int) \
                and not self.is_time_specifiers_out_of_bound(time):
            self._actions.pop(time)
            return True
        elif action is None and isinstance(time,int) and not self.is_time_specifiers_out_of_bound(time):
            self._actions.pop(time)
            return True
        else:
            return False

    def contains(self, time, action):
        """
        Returns if an action is contained in the plan at a specified time specifier.
        :param time: the time specifier.
        :param action: the action.
        :return: <code>true</code> if the specified action is contained in the plan at the specified time specifier;
         <code>false</code> otherwise.
        """
        return not(self.is_time_specifiers_out_of_bound(time) and self._actions[time])

    def clear(self):
        """
        Removes all the actions of the plan.
        """
        self._actions.clear()

    def equals(self, obj):
        """
        Returns if the plan is equal to an other object. A plan is equal to an other object if the object is an instance
        of the same class and have the same action at the same time specifier.
        :param obj: the object to be compared.
        :return: <code>true</code> if this plan is equal to the specified object; <code>false</code> otherwise.
        """
        if obj is not None and isinstance(obj, SequentialPlan) and self._actions == obj.actions:
            return True
        else:
            return False

    def is_time_specifiers_out_of_bound(self, time):
        """
        Returns if a specified time specifier is invalid. Formally a time specifier is invalid if it is less
        than 0 or greater than the size of the plan.
        :param time: the time stamp.
        :return: <code>true</code> if the specified time specifier is out of bound; <code>false</code> otherwise.
        """
        if isinstance(time, int):
            return time < 0 or time > len(self.actions())

