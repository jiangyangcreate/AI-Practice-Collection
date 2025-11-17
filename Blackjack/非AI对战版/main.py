"""
游戏规则:
1.目标是让手牌点数尽可能接近21点但不超过(超过会判负)
2.A可以算作1，J/Q/K算作10点[ok]
3.两张牌总和为21点称为'黑杰克'，赔率1.5倍庄家小于17点必须要牌，大于等于17点停牌
4.点数相同为平局，退还赌注。
"""

import random
from dataclasses import dataclass
from typing import List, Optional
from decimal import Decimal, InvalidOperation, getcontext

# 货币精度设置（两位小数）
getcontext().prec = 28

# 扑克牌花色与点数
NUMBERS: List[str] = ["A", "2", "3", "4", "5", "6", "7", "8", "9", "10", "J", "Q", "K"]
COLORS: List[str] = ["♠", "♥", "♦", "♣"]

# 常量与魔法值收敛
DEALER_STAND_VALUE: int = 17
HOUSE_BANKROLL: Decimal = Decimal("100000000000000")
# 赔率（传入 win(amount) 的 amount 为实际发放金额：包含返还本金）
PAYOUT_NORMAL: Decimal = Decimal("2")  # 正常胜：返还 2x 本金（含本金）
PAYOUT_BLACKJACK: Decimal = Decimal("2.5")  # 黑杰克胜：返还 2.5x 本金（含本金）
PAYOUT_DEALER_BLACKJACK_LOSE: Decimal = Decimal("0.5")  # 庄家 21 玩家输 0.5x
PAYOUT_TIE_PLAYER_FEWER_CARDS_WIN: Decimal = Decimal("2.5")  # 双 21 且玩家更少牌
PAYOUT_TIE_PLAYER_MORE_OR_EQUAL_CARDS_LOSE: Decimal = Decimal(
    "0.5"
)  # 双 21 且玩家牌不更少


# 业务异常：用于替代库内 sys.exit
class GameOver(Exception):
    def __init__(self, message: str):
        super().__init__(message)
        self.message = message


@dataclass(frozen=True)
class Card:
    """
    单张扑克牌，包含点数与花色。
    """

    number: str
    color: str

    def __str__(self) -> str:
        return f"{self.color}{self.number}"

    # def __add__(self, other):
    #     sums = 0
    #
    #     for card in [self, other]:
    #         if card.number == "A":
    #             sums += 1
    #         elif card.number in ["J","Q","K"]:
    #             sums += 10
    #         else:
    #             sums += int(card.number)
    #     return sums
    @staticmethod
    def get_value(cards: List["Card"]) -> int:
        """
        根据 21 点规则计算一组牌的点数。
        A 优先按 11 计入，若超过 21 则降为 1（每有一张 A 可降一次）。
        """

        numbers: List[str] = [card.number for card in cards]
        value = 0
        aces = 0
        for num in numbers:
            if num == "A":
                aces += 1
                value += 11
            elif num in ["J", "Q", "K"]:
                value += 10
            else:
                value += int(num)

        while value > 21 and aces > 0:
            value -= 10
            aces -= 1
        return value


class Deck:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.cards: List[Card] = [
            Card(number, color) for number in NUMBERS for color in COLORS
        ]

        random.shuffle(self.cards)

    def deal_card(self) -> Card:
        if len(self.cards) == 0:
            self.reset()
        return self.cards.pop()


class Player:
    def __init__(self, name: str, money: Decimal):
        self.name: str = name
        self.money: Decimal = money
        self.cards: List[Card] = []
        self.bet_amount: Optional[Decimal] = None

    def add_card(self, card: Card) -> None:
        self.cards.append(card)

    # 下注
    def bet(self, amount: Decimal) -> bool:
        if amount > self.money:
            print(f"{self.name} 没有足够的资金")
            return False
        self.money -= amount
        self.bet_amount = amount
        return True

    def print_money(func):
        def wrapper(self, amount: Decimal):
            func(self, amount)
            if func.__name__ == "win":
                print(
                    f"{self.name} 赢了{amount - self.bet_amount}元，当前余额{self.money}元"
                )
            elif func.__name__ == "lose":
                print(
                    f"{self.name} 输了{amount + self.bet_amount}元，当前余额{self.money}元"
                )
            return None

        return wrapper

    # 赢钱
    @print_money
    def win(self, amount: Decimal) -> None:
        self.money += amount

    # 输钱
    @print_money
    def lose(self, amount: Decimal) -> None:
        self.money = max(self.money - amount, Decimal("0"))
        if self.money == 0:
            # 不在库内退出进程，改为抛出业务异常
            raise GameOver(f"{self.name} 没有足够的资金 游戏结束")

    def show_hand_cards(self) -> List[str]:
        return [str(card) for card in self.cards]


class Game:
    def __init__(self, player: Player):
        self.player = player
        self.dealer = Player("庄家", HOUSE_BANKROLL)
        self.deck = Deck()

    # 下注
    def bet(self) -> Decimal:
        while True:
            raw = input("请输入你的下注金额: ")
            try:
                amount = Decimal(raw)
            except (InvalidOperation, ValueError):
                print("请输入有效的金额")
                continue
            if amount <= Decimal("0"):
                print("请输入大于0的金额")
                continue

            if self.player.bet(amount) is False:
                continue
            else:
                return amount

    # 发牌
    def deal(self) -> None:
        self.player.add_card(self.deck.deal_card())
        self.player.add_card(self.deck.deal_card())
        self.dealer.add_card(self.deck.deal_card())
        self.dealer.add_card(self.deck.deal_card())

    # 用户选择要牌
    def player_hit(self) -> None:
        print(f"玩家的牌是{self.player.show_hand_cards()}")
        while True:
            choice = input(f"是否要牌(y/n):").lower()
            if choice == "y":
                self.player.add_card(self.deck.deal_card())
                player_value = Card.get_value(self.player.cards)
                # 用光标控制序列（ANSI ESC）上移一行到行首
                print(
                    f"\x1b[1F\x1b[2K\x1b[1F\x1b[2K"
                    + f"玩家的牌是{self.player.show_hand_cards()}"
                )
                if player_value > 21:
                    break
            elif choice == "n":
                break
            else:
                print("请输入y或n")
                continue
        print()

    # 庄家自动要牌
    def dealer_hit(self) -> None:
        dealer_value = Card.get_value(self.dealer.cards)
        while True:
            if dealer_value < DEALER_STAND_VALUE:
                self.dealer.add_card(self.deck.deal_card())
                dealer_value = Card.get_value(self.dealer.cards)
            elif dealer_value >= DEALER_STAND_VALUE:
                break

    def compare(self, amount: Decimal) -> None:
        """
        停牌-比较大小（规则对齐）：
        - 双 21：玩家牌更少赢 2.5x；否则输 0.5x
        - 仅玩家 21：赢 2.5x
        - 仅庄家 21：输 0.5x
        - 任意一方爆牌：爆牌方输（庄家爆：玩家赢 2x；玩家爆：输 0）
        - 点数相等（非 21）：平局退还本金（1x）
        - 其他：点数更大者胜（玩家胜赢 2x）
        """
        print(f"庄家的牌是{self.dealer.show_hand_cards()}")
        print(f"玩家的牌是{self.player.show_hand_cards()}")
        player_value = Card.get_value(self.player.cards)
        dealer_value = Card.get_value(self.dealer.cards)

        if player_value > 21:
            self.player.lose(Decimal("0"))
        elif dealer_value > 21:
            self.player.win(amount * PAYOUT_NORMAL)

        # 当玩家和庄家都是21点时，比较牌的数量，玩家数量少赢，否则输
        elif (player_value == 21) and (dealer_value == 21):
            if len(self.player.cards) >= len(self.dealer.cards):
                self.player.lose(amount * PAYOUT_TIE_PLAYER_MORE_OR_EQUAL_CARDS_LOSE)
            elif len(self.player.cards) < len(self.dealer.cards):
                self.player.win(amount * PAYOUT_TIE_PLAYER_FEWER_CARDS_WIN)

        # 非21点且点数相同：平局，退还本金
        elif player_value == dealer_value:
            self.player.win(amount)
        # 当玩家是21点时，赢1.5倍
        elif player_value == 21:
            self.player.win(amount * PAYOUT_BLACKJACK)
        # 当庄家是21点时，输1.5倍
        elif dealer_value == 21:
            self.player.lose(amount * PAYOUT_DEALER_BLACKJACK_LOSE)

        # 正常无倍数输赢
        elif player_value > dealer_value:
            self.player.win(amount * PAYOUT_NORMAL)

        else:
            self.player.lose(Decimal("0"))

    def reset(self) -> None:
        self.player.cards = []
        self.dealer.cards = []

    def start(self) -> None:
        while True:
            self.reset()
            amount = self.bet()
            self.deal()
            self.dealer_hit()
            # 开局时展示庄家的第一张牌，其他以 ??? 隐藏
            dealer_preview = [str(self.dealer.cards[0])] + [
                "???" for _ in range(len(self.dealer.cards) - 1)
            ]
            print(f"庄家的牌是{dealer_preview}")
            self.player_hit()
            self.compare(amount)
            if self.player.money == 0:
                print("你破产了，游戏结束")
                break
            elif self.dealer.money == 0:
                print("庄家破产了，你赢了")
                break
            else:
                continue


def main() -> None:
    print(f"""
{"=" * 20}
游戏规则:
1.目标是让手牌点数尽可能接近21点但不超过
2.A可以算作1或11点，J/Q/K算作10点
3.两张牌总和为21点称为'黑杰克'，赔率1.5倍庄家小于17点必须要牌，大于等于17点停牌
4.点数相同为平局，退还赌注
{"=" * 20}
""")

    user_name = input("请输入你的名字: ").replace(" ", "") or "玩家"
    print(f'欢迎 "{user_name}" 来到21点游戏!')

    while True:
        raw_money = input("请输入你的初始资金: ")
        try:
            money = Decimal(raw_money)
        except (InvalidOperation, ValueError):
            print("请输入有效的金额")
            continue

        if money <= Decimal("0"):
            print("请输入大于0的金额")
            continue
        break
    print(f"你有{money}元")
    game = Game(Player(user_name, money))
    try:
        game.start()
    except GameOver as e:
        print(e.message)
    finally:
        print(f"庄家: {game.dealer.show_hand_cards()}")
        print(f"玩家: {game.player.show_hand_cards()}")


if __name__ == "__main__":
    main()
