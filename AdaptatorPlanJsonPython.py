from Exp import Exp
from CondExp import CondExp
from Exp import Exp
from Fluents import Fluents
from Action import Action
from SequentialPlan import SequentialPlan
from Plan import Plan
import json
from pprint import pprint


def unicodes_convert_to_str(unicode_chaine_or_list):
    """
    This function allow us to convert a unicode string or a list of unicode strings into a str or a list of str
    in order to use them to create Fluents
    :param unicode_chaine_or_list: either a list of unicode string or a unicode string that need to be convert into str
    :return: a list of str or a str
    """
    if isinstance(unicode_chaine_or_list, unicode):
        return unicode_chaine_or_list.encode('utf-8')
    elif isinstance(unicode_chaine_or_list, list):
        list_str = list()
        for elem_list in unicode_chaine_or_list:
            if isinstance(elem_list, unicode):
                elem_list = elem_list.encode('utf-8')
                list_str.append(elem_list)
            else:
                raise SyntaxError("Error function byteify in class AdaptatorPlanJsonPython.py,"
                                  " not unicode value in the list provided")
        return list_str
    else:
        raise SyntaxError("error function byteify in class AdaptatorPlanJsonPython.py,"
                          " not a unicode type or a list of unicode type")


def split_chain_str_space(chaine_str):
    """
    This function is designed in order to split the json's string of a fluents and get in a list the predicate
    and the parameters.
    :param chaine_str: the fluents in a format of a single str
    :return: a list with the component of chaine_str, the 1st comoponent is the predicat and the others are
    the parameters
    """
    liste_chain_split = list()
    if isinstance(chaine_str,str):
        liste_chain_split = chaine_str.split()
    else:
        raise SyntaxError("Error in function split_chain_str_space in class AdaptatorPlanJsonPython.py,"
                          " the value of the parameter is not a str")
    return liste_chain_split


def delete_parenthesis_str(chaine_str):
    """
    This function deletes the parenthesis of a string, in our case it remove the parenthesis of the json string that
    define a fluents
    :param chaine_str: the json string that define a fluent
    :return: chaine_str without the parenthesis
    """
    if isinstance(chaine_str,str):
        new_str = chaine_str.replace("(", "")
        new_str_final = new_str.replace(")", "")
        return new_str_final
    else:
        raise SyntaxError("Error in function delete_parentesis_str in class AdaptatorPlanJsonPython.py,"
                          " the value of the parameter is not a str")


def get_precondition(action):
    """
    This function return a Exp object from a action json object
    :param action: action json object
    :return: a Exp
    """
    fluents_positives_list = get_fluents_from_json_object(action["Preconditions"]["Positives"])
    fluents_negatives_list = get_fluents_from_json_object(action["Preconditions"]["Negatives"])

    preconditions = Exp(fluents_positives_list, fluents_negatives_list)

    return preconditions


def get_fluents_from_json_object(json_object):
    """
    This function return a liste of fluents from a json object that contains positives or negatives
    :param json_object: positives or negatives in json object
    :return: a list of fluents
    """
    # We create a list of fluents we want to return
    liste_fluents = list()

    # we convert the json array we have in parameterinto a str array (the json array in in unicode
    liste_str_converted = unicodes_convert_to_str(json_object)

    # This loop will create the fluents and add them into the fluents list we created earlier
    for elem_liste_str_converted in liste_str_converted:

        # We split the chain we get when we iterate on the liste_str_converted in order to create the Fluents
        # We want to extract the predicate and the
        liste_split_str_liste_str_converted = split_chain_str_space(delete_parenthesis_str(elem_liste_str_converted))
        if len(liste_split_str_liste_str_converted) != 0:
            fluent = Fluents(liste_split_str_liste_str_converted[0], liste_split_str_liste_str_converted[1:])
            liste_fluents.append(fluent)

    return liste_fluents


def get_cond_exp_condition(json_object):
    """
    This function return a cond_exp's condition for an action jsonb object
    :param json_object: json condition object
    :return: a cond_exp's condition object
    """

    # We extract the positives & negatives from the json's condition object
    liste_condition_positives = get_fluents_from_json_object(json_object["Condition"]["Positives"])
    liste_condition_negatives = get_fluents_from_json_object(json_object["Condition"]["Negatives"])

    # We create the Exp for the condition
    condition = Exp(liste_condition_positives, liste_condition_negatives)

    return condition


def get_cond_exp_effect(json_object):
    """
    This function return a cond_exp's effect for an action jsonb object
    :param json_object: json effect object
    :return: a cond_exp's effect object
    """
    # We extract the positives & negatives from the json's effect object
    liste_effect_positives = get_fluents_from_json_object(json_object["Effect"]["Positives"])
    liste_effect_negatives = get_fluents_from_json_object(json_object["Effect"]["Negatives"])

    # We create the Exp for the effect
    effect = Exp(liste_effect_positives, liste_effect_negatives)

    return effect


def get_cond_exp(action):
    """
    This function return the list of CondExp from a acton object
    :param action: json action object
    :return: a list of CodnExp
    """

    # This is the list that we want to return
    liste_cond_exp = list()

    # This is the list of the cond_exp in json format
    liste_cond_exp_json = action["Condition_Expressions"]

    # This loop will create a CondExp object and append it to the list of CondExp we want to return in this function
    for elem_liste_cond_exp in liste_cond_exp_json:

        condition = get_cond_exp_condition(elem_liste_cond_exp)
        effect = get_cond_exp_effect(elem_liste_cond_exp)

        cond_exp = CondExp(effect, condition)

        liste_cond_exp.append(cond_exp)

    return liste_cond_exp


def get_sequential_plan(json_object):
    """
    This function return a sequential_plan created from the informations extracted from the json file.
    :param json_object: the json at his root
    :return: sequential_plan
    """

    # On affiche les timeSpecifiers
    pprint(json_object["timeSpecifiers"])

    # On les stocks
    time_specifiers = data["timeSpecifiers"]

    # Liste qui contiendra les objets actions
    actions = list()

    # Liste qui contiendra les objects actions en json pour permettre de creer les objects actions
    # pour la liste actions
    actions_json = list()

    # Boucle permettant de recuperer les objects json actions
    for elem_temps in time_specifiers:
        actions_json.append(data["Action" + " " + repr(elem_temps)])

    # Boucle permettant de creer un object action est de l'ajouter a la liste d'actions
    for elem_action in actions_json:
        # We get the precondition from the json
        precondition = get_precondition(elem_action)

        # We get the list of CondExp from the Json
        list_cond_exp = get_cond_exp(elem_action)

        # We create an action
        action = Action(unicodes_convert_to_str(elem_action["Names"]),
                        unicodes_convert_to_str(elem_action["Parameters"]), precondition, list_cond_exp)

        # We add the action to the list of actions
        actions.append(action)

    plan = SequentialPlan(actions)

    return plan


def load_json_file(name_of_the_json_file):
    """
    This function load the json in an object in order to extract the data from it
    :param name_of_the_json_file: it's the name of the json file generated by the java part of the module,
     it should be a str
    :return: a json object that will be used in order to get information about the plan
    """
    if isinstance(name_of_the_json_file, str):
        # We open the json file and put it in a variable called data
        with open(name_of_the_json_file) as data_file:
            json_data = json.load(data_file)
            return json_data
    else:
        raise SyntaxError("Error in function load_json_file in AdaptatorPlanJsonPython.py,"
                          "the value of the parameter is not a str")


if __name__ == "__main__":

    print ("Debut de la recuperation des donnees : \n")

    data = load_json_file("plan.json")

    # If the plan from the json file is a Sequential Plan
    if data["Type_de_plan"] == 1:
        print ("Sequential Plan :")

        sequential_plan = get_sequential_plan(data)

        # Partie de test

        pprint(sequential_plan.cost())
        pprint(sequential_plan.makespan())
        pprint(sequential_plan.size())

        pprint(sequential_plan.is_empty())
        pprint(sequential_plan.is_time_specifiers_out_of_bound(17))
        liste = sequential_plan.get_actions_set(3)
        for elem in liste:
            elem.affiche()




