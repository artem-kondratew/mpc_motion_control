#!/usr/bin/env python

"""Модуль тестирования функционала"""

import arguments
import command
import misc
import parameters

from unittest.mock import call

__author__ = "Сахаров Дмитрий <saharov.dd@starline.ru>"
__copyright__ = "StarLine LLC, 2024"


def test_create_callback_1(mocker):
    """Успешный вызов без параметров"""
    callback = command.create_callback("echo hi")
    mocker.patch("misc.execute")
    callback()
    misc.execute.assert_called_once_with("echo hi")


def test_create_callback_2(mocker):
    """Успешный вызов с 1 параметром"""
    callback = command.create_callback("echo __placeholder_1__")
    mocker.patch("misc.execute")
    callback(15)
    misc.execute.assert_called_once_with("echo 15")


def test_create_callback_3(mocker):
    """Успешный вызов с 2 параметрами"""
    callback = command.create_callback("echo __placeholder_1__ __placeholder_2__")
    mocker.patch("misc.execute")
    callback(15, 20)
    misc.execute.assert_called_once_with("echo 15 20")


def test_create_callback_4(mocker):
    """Вызов без параметра, а ожидался 1"""
    callback = command.create_callback("echo __placeholder_1__")
    mocker.patch("misc.execute")
    callback()
    misc.execute.assert_called_once_with("echo ")


def test_create_callback_5(mocker):
    """Вызов без параметра, а ожидался 1"""
    callback = command.create_callback("echo __placeholder_2__")
    mocker.patch("misc.execute")
    callback(15)
    misc.execute.assert_called_once_with("echo ")


def test_create_callback_6(mocker):
    """Вызов без параметра, а ожидался 1"""
    callback = command.create_callback("echo __placeholder_2__")
    mocker.patch("misc.execute")
    callback(15, 20)
    misc.execute.assert_called_once_with("echo 20")


def test_create_callback_7(mocker):
    """Лишний параметр"""
    callback = command.create_callback("echo __placeholder_1__")
    mocker.patch("misc.execute")
    callback(15, 20)
    misc.execute.assert_called_once_with("echo 15")


def test_create_callback_8(mocker):
    """Неправильная нумерация параметров"""
    callback = command.create_callback("echo __placeholder_0__")
    mocker.patch("misc.execute")
    callback(15)
    misc.execute.assert_called_once_with("echo ")


def test_create_callback_9(mocker):
    """Неправильная нумерация параметров"""
    callback = command.create_callback("echo __placeholder_0__")
    mocker.patch("misc.execute")
    callback()
    misc.execute.assert_called_once_with("echo ")


def test_parse_services_1():
    """Парсинг сервисов"""
    config = {"services": {"name_1": [], "name_2": []}}

    expected_services = ["name_1", "name_2"]
    actual_services = parameters.get_services(config)
    assert actual_services == expected_services


def test_parse_services_2():
    """Парсинг сервисов"""
    config = {"services": {"name_2": [], "name_1": []}}

    expected_services = ["name_1", "name_2"]
    actual_services = parameters.get_services(config)
    assert actual_services == expected_services


def test_parse_services_3():
    """Парсинг сервисов"""
    config = {"services": {}}

    expected_services = []
    actual_services = parameters.get_services(config)
    assert actual_services == expected_services


def test_parse_profiles_1():
    """Парсинг профилей"""
    config = {"services": {"name_1": {"profiles": ["a", "b"]}}}

    expected_profiles = ["a", "b"]
    actual_profiles = parameters.get_profiles(config)
    assert actual_profiles == expected_profiles


def test_parse_profiles_2():
    """Парсинг профилей"""
    config = {"services": {"name_1": {"profiles": ["b", "a"]}}}

    expected_profiles = ["a", "b"]
    actual_profiles = parameters.get_profiles(config)
    assert actual_profiles == expected_profiles


def test_parse_profiles_3():
    """Парсинг профилей"""
    config = {
        "services": {
            "name_1": {"profiles": ["b", "c"]},
            "name_2": {"profiles": ["d", "a", "c"]},
        }
    }

    expected_profiles = ["a", "b", "c", "d"]
    actual_profiles = parameters.get_profiles(config)
    assert actual_profiles == expected_profiles


def test_parse_profiles_4():
    """Парсинг профилей"""
    config = {
        "services": {
            "name_1": {"profiles": []},
            "name_2": {"profiles": ["d", "a", "c"]},
        }
    }

    expected_profiles = ["a", "c", "d"]
    actual_profiles = parameters.get_profiles(config)
    assert actual_profiles == expected_profiles


def test_parse_profiles_5():
    """Парсинг профилей"""
    config = {"services": {"name_1": {}, "name_2": {"profiles": ["d", "a", "c"]}}}
    expected_profiles = ["a", "c", "d"]
    actual_profiles = parameters.get_profiles(config)
    assert actual_profiles == expected_profiles


def test_parse_profiles_6():
    """Парсинг профилей"""
    config = {"services": {"name_1": {}, "name_2": {"profiles": []}}}

    expected_profiles = []
    actual_profiles = parameters.get_profiles(config)
    assert actual_profiles == expected_profiles


def test_parse_profiles_7():
    """Парсинг профилей"""
    config = {"services": {"name_1": {}, "name_2": {}}}

    expected_profiles = []
    actual_profiles = parameters.get_profiles(config)
    assert actual_profiles == expected_profiles


def test_parse_profiles_8():
    """Парсинг профилей"""
    config = {
        "services": {
            "name_1": {},
        }
    }

    expected_profiles = []
    actual_profiles = parameters.get_profiles(config)
    assert actual_profiles == expected_profiles


def test_command_eq_1():
    """Проверка __eq__ у команды"""
    c1 = command.Command()
    c1.name = "hi"
    c1.aliases = ["a", "b"]
    c1.parameters = [["p1"], ["p2", "p3"]]
    assert c1 == c1


def test_command_eq_2():
    """Проверка __eq__ у команды"""
    c1 = command.Command()
    c1.name = "hi"
    c1.aliases = ["a", "b"]
    c1.parameters = [["p1"], ["p2", "p3"]]

    c2 = command.Command()
    c2.name = "hi"
    c2.aliases = ["a", "b"]
    c2.parameters = [["p1"], ["p2", "p3"]]
    assert c1 == c2


def test_command_eq_3():
    """Проверка __eq__ у команды с разными именами"""
    c1 = command.Command()
    c1.name = "hi1"
    c1.aliases = ["a", "b"]
    c1.parameters = [["p1"], ["p2", "p3"]]

    c2 = command.Command()
    c2.name = "hi"
    c2.aliases = ["a", "b"]
    c2.parameters = [["p1"], ["p2", "p3"]]
    assert c1 != c2


def test_command_eq_4():
    """Проверка __eq__ у команды с разными алиасами"""
    c1 = command.Command()
    c1.name = "hi"
    c1.aliases = ["a", "c"]
    c1.parameters = [["p1"], ["p2", "p3"]]

    c2 = command.Command()
    c2.name = "hi"
    c2.aliases = ["a", "b"]
    c2.parameters = [["p1"], ["p2", "p3"]]
    assert c1 != c2


def test_command_eq_5():
    """Проверка __eq__ у команды с разными параметрами"""
    c1 = command.Command()
    c1.name = "hi"
    c1.aliases = ["a", "b"]
    c1.parameters = [["p1"], ["p2", "p3"]]

    c2 = command.Command()
    c2.name = "hi"
    c2.aliases = ["a", "b"]
    c2.parameters = [["p2"], ["p2", "p3"]]
    assert c1 != c2


def test_command_eq_6():
    """Проверка __eq__ у команды с разными shell cmd"""
    c1 = command.Command()
    c1.name = "hi"
    c1.aliases = ["a", "b"]
    c1.parameters = [["p1"], ["p2", "p3"]]
    c1.commands = ["123"]

    c2 = command.Command()
    c2.name = "hi"
    c2.aliases = ["a", "b"]
    c2.parameters = [["p1"], ["p2", "p3"]]
    c1.commands = ["234"]
    assert c1 == c2


def test_parse_commands_1(mocker):
    """Базовая проверка парсера команд"""
    compose_config = {
        "services": {
            "tf": {"profiles": {"all", "tf"}},
            "sensing": {"profiles": {"all", "gnss"}},
        }
    }

    run_config = {
        "test": {
            "aliases": ["up", "run"],
            "parameters": ["__profile__", "__service__"],
            "commands": ["echo __placeholder_1__", "echo __placeholder_2__"],
        }
    }

    c = command.Command()
    c.name = "test"
    c.aliases = ["up", "run"]
    c.parameters = [["all", "gnss", "tf"], ["sensing", "tf"]]

    expected_commands = [c]
    actual_commands = command.parse_commands(compose_config, run_config)
    assert actual_commands == expected_commands

    assert len(actual_commands) == 1
    assert len(actual_commands[0].commands) == 2

    # проверяем что callback-и вызываются как нужно
    mocker.patch("misc.execute")
    actual_commands[0].execute("all", "sensing")
    misc.execute.assert_has_calls(
        [call("echo all"), call("echo sensing")], any_order=False
    )
    assert misc.execute.call_count == 2


def test_parse_commands_2(mocker):
    """Тут проверка того, что вместо списка команд мы отправили 1 команду строкой"""
    compose_config = {
        "services": {
            "tf": {"profiles": {"all", "tf"}},
            "sensing": {"profiles": {"all", "gnss"}},
        }
    }

    run_config = {
        "test": {
            "aliases": ["up", "run"],
            "parameters": ["__profile__", "__service__"],
            "commands": "echo __placeholder_1__",
        }
    }

    c = command.Command()
    c.name = "test"
    c.aliases = ["up", "run"]
    c.parameters = [["all", "gnss", "tf"], ["sensing", "tf"]]

    expected_commands = [c]
    actual_commands = command.parse_commands(compose_config, run_config)
    assert actual_commands == expected_commands

    assert len(actual_commands) == 1
    assert len(actual_commands[0].commands) == 1

    # проверяем что callback-и вызываются как нужно
    mocker.patch("misc.execute")
    actual_commands[0].execute("all", "sensing")
    misc.execute.assert_called_once_with("echo all")


def test_parse_commands_3(mocker):
    """Тут проверка того, что вместо списка параметров мы отправили 1 параметр строкой"""
    compose_config = {
        "services": {
            "tf": {"profiles": {"all", "tf"}},
            "sensing": {"profiles": {"all", "gnss"}},
        }
    }

    run_config = {
        "test": {
            "aliases": ["up", "run"],
            "parameters": "__profile__",
            "commands": ["echo __placeholder_1__", "echo __placeholder_1__"],
        }
    }

    c = command.Command()
    c.name = "test"
    c.aliases = ["up", "run"]
    c.parameters = [["all", "gnss", "tf"]]

    expected_commands = [c]
    actual_commands = command.parse_commands(compose_config, run_config)
    assert actual_commands == expected_commands

    assert len(actual_commands) == 1
    assert len(actual_commands[0].commands) == 2

    # проверяем что callback-и вызываются как нужно
    mocker.patch("misc.execute")
    actual_commands[0].execute("all")
    misc.execute.assert_has_calls([call("echo all"), call("echo all")], any_order=False)
    assert misc.execute.call_count == 2


def test_parse_commands_4(mocker):
    """Парсер команд, но вызываем с числом параметров больше, чем ожидалось"""
    compose_config = {
        "services": {
            "tf": {"profiles": {"all", "tf"}},
            "sensing": {"profiles": {"all", "gnss"}},
        }
    }

    run_config = {
        "test": {
            "aliases": ["up", "run"],
            "parameters": ["__profile__", "__service__"],
            "commands": ["echo __placeholder_1__", "echo __placeholder_2__"],
        }
    }

    c = command.Command()
    c.name = "test"
    c.aliases = ["up", "run"]
    c.parameters = [["all", "gnss", "tf"], ["sensing", "tf"]]

    expected_commands = [c]
    actual_commands = command.parse_commands(compose_config, run_config)
    assert actual_commands == expected_commands

    assert len(actual_commands) == 1
    assert len(actual_commands[0].commands) == 2

    # проверяем что callback-и вызываются как нужно
    mocker.patch("misc.execute")
    actual_commands[0].execute("all", "sensing", "dddd")
    misc.execute.assert_has_calls(
        [call("echo all"), call("echo sensing")], any_order=False
    )
    assert misc.execute.call_count == 2


def test_parse_commands_5(mocker):
    """Проверка парсера команд с описанием"""
    compose_config = {
        "services": {
            "tf": {"profiles": {"all", "tf_spec"}},
            "sensing": {"profiles": {"all", "gnss"}},
        }
    }

    run_config = {
        "test": {
            "desc": "some test desc",
            "aliases": ["up", "run"],
            "parameters": ["__profile__", "__service__"],
            "commands": ["echo __placeholder_2__", "echo __placeholder_1__"],
        }
    }

    c = command.Command()
    c.name = "test"
    c.description = "some test desc"
    c.aliases = ["up", "run"]
    c.parameters = [["all", "gnss", "tf_spec"], ["sensing", "tf"]]

    expected_commands = [c]
    actual_commands = command.parse_commands(compose_config, run_config)
    assert actual_commands == expected_commands

    assert len(actual_commands) == 1
    assert len(actual_commands[0].commands) == 2

    # проверка description
    assert actual_commands[0].description == expected_commands[0].description

    # проверяем что callback-и вызываются как нужно
    mocker.patch("misc.execute")
    actual_commands[0].execute("tf_spec", "sensing")
    misc.execute.assert_has_calls(
        [call("echo sensing"), call("echo tf_spec")], any_order=False
    )
    assert misc.execute.call_count == 2


def test_parse_commands_6(mocker):
    """Проверка парсера команд, с вызовом callback с параметрами в другом порядке"""
    compose_config = {
        "services": {
            "tf": {"profiles": {"all", "tf_spec"}},
            "sensing": {"profiles": {"all", "gnss"}},
        }
    }

    run_config = {
        "test": {
            "desc": "some test desc",
            "aliases": ["up", "run"],
            "parameters": ["__profile__", "__service__"],
            "commands": ["echo __placeholder_2__", "echo __placeholder_1__"],
        }
    }

    c = command.Command()
    c.name = "test"
    c.description = "some test desc"
    c.aliases = ["up", "run"]
    c.parameters = [["all", "gnss", "tf_spec"], ["sensing", "tf"]]

    expected_commands = [c]
    actual_commands = command.parse_commands(compose_config, run_config)
    assert actual_commands == expected_commands

    assert len(actual_commands) == 1
    assert len(actual_commands[0].commands) == 2

    # проверка description
    assert actual_commands[0].description == expected_commands[0].description

    # проверяем что callback-и вызываются как нужно
    #  тут порядок параметров уже не должен иметь значения, т.к. заполняться они должны "верхним" уровнем
    #  т.е. тут все выполнится в любом случае
    mocker.patch("misc.execute")
    actual_commands[0].execute("sensing", "tf_spec")
    misc.execute.assert_has_calls(
        [call("echo tf_spec"), call("echo sensing")], any_order=False
    )
    assert misc.execute.call_count == 2


def test_argument_parser_1():
    """Тест парсера аргументов командной строки"""
    c = command.Command()
    c.name = "test"
    c.aliases = ["up", "run"]
    c.parameters = [["all", "gnss", "tf"], ["sensing", "tf"]]

    parser = arguments.register_arguments([c])
    args = parser.parse_args(["test", "all", "tf"])
    placeholders = arguments.parse_placeholders(args)

    assert len(placeholders) == 2
    assert args.cmd == "test"
    assert placeholders[0] == "all"
    assert placeholders[1] == "tf"
    # TODO: проверить тест вывода описания параметров
