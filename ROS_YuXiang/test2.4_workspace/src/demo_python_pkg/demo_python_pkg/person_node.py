class PersonNode:
    def __init__(self, name: str, age: int) -> None:
        print(f"{name} is {age} years old")
        self.name = name
        self.age = age

    def eat(self, food: str) -> None:
        print(f"{self.name} is eating {food}")

def main():
    node = PersonNode("Alice", 18)
    node.eat("apple")
        