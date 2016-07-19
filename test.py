
from Exp import Exp
from CondExp import CondExp
from Exp import Exp
from Fluents import Fluents
from Action import Action
from SequentialPlan import SequentialPlan


if __name__ == "__main__":

    """ Creation of Fluents / Exp and Cond Exp in order to test the Action class """
    fluents_positives_exp = Fluents("on", ["a", "b"])
    fluents_negatives_exp = Fluents("handempty")

    exp = Exp(fluents_positives_exp, fluents_negatives_exp)

    fluents_positives_exp_cond_exp_1 = Fluents("on", ["a","b"])
    fluents_negatives_exp_cond_exp_1 = Fluents("on", ["a", "g"])
    fluents_positives_exp_cond_exp_2 = Fluents("on", ["e", "f"])
    fluents_negatives_exp_cond_exp_2 = Fluents("on", ["b", "h"])

    exp_cond_exp_1 = Exp(fluents_positives_exp_cond_exp_1, fluents_negatives_exp_cond_exp_1)
    exp_cond_exp_2 = Exp(fluents_positives_exp_cond_exp_2, fluents_negatives_exp_cond_exp_2)

    cond_exp = CondExp(exp_cond_exp_1, exp_cond_exp_2)

    fluents_positives_action = Fluents("on", ["g", "h"])
    fluents_negatives_action = Fluents("on", ["i", "j"])

    exp_action = Exp(fluents_positives_action, fluents_negatives_action)

    action = Action("unstack", ["a", "b"], exp_action, cond_exp)

    fluents_positives_add_1 = Fluents("on", ["n", "v"])
    fluents_negatives_add_2 = Fluents("on", ["w", "x"])
    fluents_positives_add_3 = Fluents("on", ["h", "y"])
    fluents_negatives_add_4 = Fluents("on", ["l", "m"])

    exp_add_1 = Exp(fluents_positives_add_1, fluents_negatives_add_2)
    exp_add_2 = Exp(fluents_positives_add_3, fluents_negatives_add_4)

    cond_exp_add = CondExp(exp_add_1, exp_add_2)

    action.add_condition_exp(cond_exp_add)

    second_action = Action("stack", ["b","a"], exp_action, cond_exp)

    sequential_plan = SequentialPlan([action, second_action])

    for elem in sequential_plan.actions():
        print elem.affiche()


