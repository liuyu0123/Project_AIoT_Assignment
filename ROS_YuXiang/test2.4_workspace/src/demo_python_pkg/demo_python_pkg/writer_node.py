from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode):
    def __init__(self, name: str, age: int, book: str) -> None:
        super().__init__(name, age)
        print('WriterNode is initialized')
        self.book = book

    def write(self, book: str) -> None:
        self.books.append(book)

    def read(self, book: str) -> None:
        if book in self.books:
            print(f"{self.name} is reading {book}")

def main():
    node = WriterNode("L",18,"Death Note")
    node.eat("apple")