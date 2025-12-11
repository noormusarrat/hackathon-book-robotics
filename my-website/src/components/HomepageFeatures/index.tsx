import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  image: string;          // URL string, not component
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: '🧠 Learn the Robotic Nervous System',
    image: require('@site/static/img/Robotic Nervous System.jpg').default,
    description: (
      <>
        Master ROS 2, the foundational middleware that powers real-world humanoid control,
        communication, and sensor processing.
      </>
    ),
  },
  {
    title: '🤖 Train the AI-Robot Brain',
    image: require('@site/static/img/AI-Robot Brain.jpg').default,
    description: (
      <>
        Use NVIDIA Isaac Sim & Isaac ROS for advanced perception, SLAM, navigation, and
        photorealistic synthetic data generation.
      </>
    ),
  },
  {
    title: '🏆 Capstone: The Autonomous Humanoid',
    image: require('@site/static/img/Autonomous Humanoid.jpg').default,
    description: (
      <>
        Build a pipeline where a humanoid robot receives a voice command → interprets it →
        navigates a room → detects objects → manipulates them safely.
      </>
    ),
  },
];

function Feature({title, image, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img
          src={image}
          alt={title}
          className={styles.featureImg}
        />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
