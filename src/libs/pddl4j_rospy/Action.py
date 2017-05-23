#!/usr/bin/env python

from Exp import Exp
from CondExp import CondExp
from Exp import Exp
from Fluents import Fluents


class Action:
    """
    This class implements a representation of an action
    attributes :
    - name
    - duration
    - preconditions
    - effects
    """
    def __init__(self, name=None, parameters=None, preconditions=None, cond_expression=None, cost=1.0):
        """
        Constructor of the Class
        :param name: the name of the action represented by str
        :param parameters: the parameters of the actions represented by a list of str
        :param preconditions: the precondition of the action represented by a Exp object
        :param cond_expression: the condition_expression of the action represented by a CondExp object
        :param cost: the cost of the action represented by int
        """
        # If we only know the name
        if name is not None \
                and parameters is None \
                and preconditions is None \
                and cond_expression is None \
                and isinstance(name, str):
            self._name = name
            self._cost = cost

        # If we know the name and the parameters (str)
        elif name is not None \
                and parameters is not None \
                and isinstance(parameters, str)\
                and preconditions is None \
                and cond_expression is None \
                and isinstance(name, str):
            self._parameters = parameters
            self._name = name
            self._cost = cost

        # If we know the name and the parameters  (list)
        elif name is not None \
                and parameters is not None \
                and isinstance(parameters, list) \
                and preconditions is None \
                and cond_expression is None \
                and isinstance(name, str):

            self._parameters = list()
            self._parameters = parameters
            self._name = name
            self._cost = cost

        # If we know the name, the parameters (str), the preconditions and the cond_expression
        elif name is not None \
                and preconditions is not None \
                and cond_expression is not None \
                and parameters is not None \
                and Action.is_cond_exp_list(cond_expression) \
                and isinstance(preconditions, Exp) \
                and isinstance(name, str) \
                and isinstance(parameters, str):

            self._name = name
            self._preconditions = preconditions
            self._parameters = parameters
            self._cost = cost
            self._cond_expression = list()

            for elem in cond_expression:
                self._cond_expression.append(elem)

        # If we know the nale, the paramters (list), the preconditions and the cond_expression
        elif name is not None \
                and preconditions is not None \
                and cond_expression is not None \
                and parameters is not None \
                and Action.is_cond_exp_list(cond_expression) \
                and isinstance(preconditions, Exp) \
                and isinstance(name, str) \
                and isinstance(parameters, list):

            self._name = name
            self._preconditions = preconditions
            self._parameters = list()
            self._cost = cost

            if isinstance(parameters,list):
                for elem in parameters:
                    self._parameters.append(elem)

            self._cond_expression = list()
            for elem in cond_expression:
                self._cond_expression.append(elem)

        # If none of that we can't create an action
        else:
            raise SyntaxError("Error in the function __init__ in class Action.py, the object was not created")

    @staticmethod
    def is_cond_exp_list(liste):
        """
        Return <code>True</code> if the list in parameter contains only CondExp objects, <code>False</code> otherwise
        :param liste: The list of CondExp
        :return: if the List is composed only of CondExp elements
        """
        bool_test = False
        if isinstance(liste, list):
            for elem in liste:
                if not isinstance(elem, CondExp):
                    bool_test = True
                    break
        elif isinstance(liste, CondExp):
            bool_test = True

        return bool

    def _get_name(self):
        """
        Action name's getter
        :return: The action name
        """
        return self._name

    def _get_preconditions(self):
        """
        Precondition's getter
        :return: The precondtion which is a Exp named self._precondtions
        """
        return self._preconditions

    def _get_cond_expressions(self):
        """
        Effects's getter
        :return: The list of CondExp named self._cond_expression
        """
        return self._cond_expression

    def _get_parameters(self):
        """
        Parameters's getter
        :return: The precondtion which is a Exp named self.parameters
        """
        return self._parameters

    def _get_cost(self):
        """
        Action cost's getter
        :return: the cost of the Action
        """
        return self._cost

    def _set_cost(self, cost):
        """
        Actions cost's setter
        :param cost: the cost of the Action
        """
        if isinstance(cost, int):
            self._cost = cost
        else:
            raise SyntaxError("Error in function _set_cost in class Action.py, the value of the parameter is not an int")

    def _set_name(self, name):
        """
        Actions name's setter
        :param name: the name of the Action
        :return:
        """
        if isinstance(name, str):
            self._name = name
        else:
            raise SyntaxError("Error in function _set_name in class Action.py, the value of the parameter is not a str")

    def _set_parameters(self, parameters):
        """
        Parameters's setter
        :param parameters: The Exp object we want to set in the Action
        """
        if isinstance(parameters,str):
            self._preconditions = parameters
        elif isinstance(parameters, list):
            for elem in parameters:
                self._parameters.append(elem)
        else:
            raise SyntaxError("Error in function _set_parameters in class Action.py, the value of the parameter "
                              "is not a str")

    def _set_preconditions(self, preconditions):
        """
        Preconditions's setter
        :param preconditions: The Exp object we want to set in the Action
        """
        if isinstance(preconditions, Exp):
            self._preconditions = preconditions
        else:
            SyntaxError("Error in function _set_precondition in class Action.py, the value of the "
                        "parameter is not a Exp")

    def _set_cond_expressions(self, cond_expression):
        """
        Effects's setter
        :param cond_expression: The list of CondExp that we want to set in the object
        """
        if Action.is_cond_exp_list(cond_expression):
            self._cond_expression = cond_expression
        else:
            SyntaxError("Error in function _set_cond_expressions in class Action.py, the value of the parameter "
                        "is not a list of cond_expression")

    def add_condition_exp(self, effects):
        """
        Add a CondExp to the list of CondExp of the action
        :param effects: the CondExp we want to add
        :return:
        """
        if isinstance(effects, list):
            if Action.is_cond_exp_list(effects):
                for elem in effects:
                    self._cond_expression.append(elem)
                    return True
        elif isinstance(effects,CondExp):
            if isinstance(self._cond_expression, list):
                self._cond_expression.append(effects)
                return True
            else:
                save = self._cond_expression
                self._cond_expression = list()
                self._cond_expression.append(save)
                self._cond_expression.append(effects)
                return True
        else:
            raise SyntaxError("Eror in function add_condition_exp in class Action.py, the parameter is not a list of "
                              "cond_expression or a cond_expression the cond_expression list was not add")

    def affiche(self):
        """
        This function display the informations of the Action
        """
        if isinstance(self._parameters, str):
            print ("Display of the Action : " + self._name + " : " + self._get_parameters() + "\n")
        elif isinstance(self._parameters, list):
            liste_affiche = ', '.join(self._parameters)
            print ("Display of the Action :  " + self._name + " : " + liste_affiche + "\n")
        else:
            raise SyntaxError("Error in function affiche in class Action.py, Can't display the parameters of the "
                              "action, error in conception")

        print ("Les preconditions : \n")
        self.preconditions.affiche()

        print ("\n La liste de CondExp : \n")
        for elem in self._cond_expression:
            elem.affiche()
        print ("\n")

    # This is the properties, that protect the use of the _gets and _sets of the class
    name = property(_get_name, _set_name)
    parameters = property(_get_parameters, _set_parameters)
    cost = property(_get_cost, _set_cost)
    preconditions = property(_get_preconditions, _set_preconditions)
    effects = property(_get_cond_expressions,_set_cond_expressions)

if __name__ == '__main__':

    # Test of the Class Action

    fluents_positives_exp = Fluents("on", "a", "b")
    fluents_negatives_exp = Fluents("handempty")

    exp = Exp(fluents_positives_exp, fluents_negatives_exp)

    fluents_positives_exp_cond_exp_1 = Fluents("on", "c", "d")
    fluents_negatives_exp_cond_exp_1 = Fluents("on", "a", "g")
    fluents_positives_exp_cond_exp_2 = Fluents("on", "e", "f")
    fluents_negatives_exp_cond_exp_2 = Fluents("on", "b", "h")

    exp_cond_exp_1 = Exp(fluents_positives_exp_cond_exp_1, fluents_negatives_exp_cond_exp_1)
    exp_cond_exp_2 = Exp(fluents_positives_exp_cond_exp_2, fluents_negatives_exp_cond_exp_2)

    cond_exp = CondExp(exp_cond_exp_1,exp_cond_exp_2)

    fluents_positives_action = Fluents("on", "g", "h")
    fluents_negatives_action = Fluents("on", "i", "j")

    exp_action = Exp(fluents_positives_action, fluents_negatives_action)

    action = Action("unstack", ["a", "b"], exp_action, cond_exp)

    fluents_positives_add_1 = Fluents("on", "n", "v")
    fluents_negatives_add_2 = Fluents("on", "w", "x")
    fluents_positives_add_3 = Fluents("on", "h", "y")
    fluents_negatives_add_4 = Fluents("on", "l", "m")

    exp_add_1 = Exp(fluents_positives_add_1, fluents_negatives_add_2)
    exp_add_2 = Exp(fluents_positives_add_3, fluents_negatives_add_4)

    cond_exp_add = CondExp(exp_add_1, exp_add_2)

    action.add_condition_exp(cond_exp_add)
    action.affiche()
